#ifndef _FTL_DATA_FRAMESTATE_HPP_
#define _FTL_DATA_FRAMESTATE_HPP_

#include <nlohmann/json.hpp>
#include <ftl/exception.hpp>
#include <ftl/codecs/channels.hpp>
#include <Eigen/Eigen>
#include <array>
#include <optional>
#include <string>

namespace ftl {
namespace data {

/**
 * Represent state that is persistent across frames. Such state may or may not
 * change from one frame to the next so a record of what has changed must be
 * kept. Changing state should be done at origin and not in the frame. State
 * that is marked as changed will then be send into a stream and the changed
 * status will be cleared, allowing data to only be sent/saved when actual
 * changes occur.
 * 
 * The provided SETTINGS type must support MsgPack and be copyable. An example
 * of settings is camera intrinsics.
 * 
 * COUNT is the number of settings channels available. For example, video state
 * has two settings channels, one for left camera and one for right camera.
 */
template <typename SETTINGS, int COUNT>
class FrameState {
	public:
	typedef SETTINGS Settings;

	FrameState();
	FrameState(FrameState &);
	FrameState(FrameState &&);

	/**
	 * Update the pose and mark as changed.
	 */
	void setPose(const Eigen::Matrix4d &pose) {
		pose_ = pose;
		changed_ += ftl::codecs::Channel::Pose;
	}

	/**
	 * Update the left settings and mark as changed.
	 */
	void setLeft(const SETTINGS &p) {
		static_assert(COUNT > 0, "No settings channel");
		settings_[0] = p;
		changed_ += ftl::codecs::Channel::Settings1;
	}

	/**
	 * Update the right settings and mark as changed.
	 */
	void setRight(const SETTINGS &p) {
		static_assert(COUNT > 1, "No second settings channel");
		settings_[1] = p;
		changed_ += ftl::codecs::Channel::Settings2;
	}

	/**
	 * Change settings using ID number. Necessary when more than 2 settings
	 * channels exist, otherwise use `setLeft` and `setRight`.
	 */
	template <int I>
	void set(const SETTINGS &p) {
		static_assert(I < COUNT, "Settings channel too large");
		settings_[I] = p;
		changed_ += __idToChannel(I);
	}

	/**
	 * Get the current pose.
	 */
	inline const Eigen::Matrix4d &getPose() const { return pose_; }

	/**
	 * Get the left settings.
	 */
	inline const SETTINGS &getLeft() const { return settings_[0]; }

	/**
	 * Get the right settings.
	 */
	inline const SETTINGS &getRight() const { return settings_[1]; }

	/**
	 * Get a modifiable pose reference that does not change the changed status.
	 * @attention Should only be used internally.
	 * @todo Make private eventually.
	 */
	inline Eigen::Matrix4d &getPose() { return pose_; }

	/**
	 * Get a modifiable left settings reference that does not change
	 * the changed status. Modifications made using this will not be propagated.
	 * @attention Should only be used internally.
	 * @todo Make private eventually.
	 */
	inline SETTINGS &getLeft() { return settings_[0]; }

	/**
	 * Get a modifiable right settings reference that does not change
	 * the changed status. Modifications made using this will not be propagated.
	 * @attention Should only be used internally.
	 * @todo Make private eventually.
	 */
	inline SETTINGS &getRight() { return settings_[1]; }

	/**
	 * Get a named config property.
	 */
	template <typename T>
	std::optional<T> get(const std::string &name) {
		try {
			return config_[name].get<T>();
		} catch (...) {
			return {};
		}
	}

	/**
	 * Helper class to specialising channel based state access.
	 * @private
	 */
	template <typename T, ftl::codecs::Channel C, typename S, int N> struct As {
		static const T &func(const ftl::data::FrameState<S,N> &t) {
			throw ftl::exception("Type not supported for state channel");
		}

		static T &func(ftl::data::FrameState<S,N> &t) {
			throw ftl::exception("Type not supported for state channel");
		}
	};

	// Specialise for pose
	template <typename S, int N>
	struct As<Eigen::Matrix4d,ftl::codecs::Channel::Pose,S,N> {
		static const Eigen::Matrix4d &func(const ftl::data::FrameState<S,N> &t) {
			return t.pose_;
		}

		static Eigen::Matrix4d &func(ftl::data::FrameState<S,N> &t) {
			return t.pose_;
		}
	};

	// Specialise for settings 1
	template <typename S, int N>
	struct As<S,ftl::codecs::Channel::Settings1,S,N> {
		static const S &func(const ftl::data::FrameState<S,N> &t) {
			return t.settings_[0];
		}

		static S &func(ftl::data::FrameState<S,N> &t) {
			return t.settings_[0];
		}
	};

	// Specialise for settings 2
	template <typename S, int N>
	struct As<S,ftl::codecs::Channel::Settings2,S,N> {
		static const S &func(const ftl::data::FrameState<S,N> &t) {
			return t.settings_[1];
		}

		static S &func(ftl::data::FrameState<S,N> &t) {
			return t.settings_[1];
		}
	};

	// Specialise for config
	template <typename S, int N>
	struct As<nlohmann::json,ftl::codecs::Channel::Configuration,S,N> {
		static const nlohmann::json &func(const ftl::data::FrameState<S,N> &t) {
			return t.config_;
		}

		static nlohmann::json &func(ftl::data::FrameState<S,N> &t) {
			return t.config_;
		}
	};

	/**
	 * Allow access to state items using a known channel number. By default
	 * these throw an exception unless specialised to accept a particular type
	 * for a particular channel. The specialisations are automatic for pose,
	 * config and SETTINGS items.
	 */
	template <typename T, ftl::codecs::Channel C>
	T &as() { return As<T,C,SETTINGS,COUNT>::func(*this); }

	/**
	 * Allow access to state items using a known channel number. By default
	 * these throw an exception unless specialised to accept a particular type
	 * for a particular channel. The specialisations are automatic for pose,
	 * config and SETTINGS items.
	 */
	template <typename T, ftl::codecs::Channel C>
	const T &as() const {
		return As<T,C,SETTINGS,COUNT>::func(*this);
	}

	/**
	 * Set a named config property. Also makes state as changed to be resent.
	 */
	template <typename T>
	void set(const std::string &name, T value) {
		config_[name] = value;
		changed_ += ftl::codecs::Channel::Configuration;
	}

	inline const nlohmann::json &getConfig() const { return config_; }

	inline nlohmann::json &getConfig() { return config_; }

	/**
	 * Check if pose or settings have been modified and not yet forwarded.
	 * Once forwarded through a pipeline / stream the changed status is cleared.
	 */
	inline bool hasChanged(ftl::codecs::Channel c) const { return changed_.has(c); }

	/**
	 * Copy assignment will clear the changed status of the original.
	 */
	FrameState &operator=(FrameState &);

	FrameState &operator=(FrameState &&);

	/**
	 * Clear the changed status to unchanged.
	 */
	inline void clear() { changed_.clear(); }

	private:
	Eigen::Matrix4d pose_;
	std::array<SETTINGS,COUNT> settings_;
	nlohmann::json config_;
	ftl::codecs::Channels<64> changed_;  // Have the state channels changed?

	static inline ftl::codecs::Channel __idToChannel(int id) {
		return (id == 0) ? ftl::codecs::Channel::Settings1 : (id == 1) ?
			ftl::codecs::Channel::Settings2 :
			static_cast<ftl::codecs::Channel>(static_cast<int>(ftl::codecs::Channel::Settings3)+(id-2));
	}
};

}
}


template <typename SETTINGS, int COUNT>
ftl::data::FrameState<SETTINGS,COUNT>::FrameState() : settings_({{0}}), config_(nlohmann::json::value_t::object) {
	pose_ = Eigen::Matrix4d::Identity();
}

template <typename SETTINGS, int COUNT>
ftl::data::FrameState<SETTINGS,COUNT>::FrameState(ftl::data::FrameState<SETTINGS,COUNT> &f) {
	pose_ = f.pose_;
	settings_ = f.settings_;
	changed_ = f.changed_;
	config_ = f.config_;
	f.changed_.clear();
}

template <typename SETTINGS, int COUNT>
ftl::data::FrameState<SETTINGS,COUNT>::FrameState(ftl::data::FrameState<SETTINGS,COUNT> &&f) {
	pose_ = f.pose_;
	settings_ = f.settings_;
	changed_ = f.changed_;
	config_ = std::move(f.config_);
	f.changed_.clear();
}

template <typename SETTINGS, int COUNT>
ftl::data::FrameState<SETTINGS,COUNT> &ftl::data::FrameState<SETTINGS,COUNT>::operator=(ftl::data::FrameState<SETTINGS,COUNT> &f) {
	pose_ = f.pose_;
	settings_ = f.settings_;
	changed_ = f.changed_;
	config_ = f.config_;
	f.changed_.clear();
	return *this;
}

template <typename SETTINGS, int COUNT>
ftl::data::FrameState<SETTINGS,COUNT> &ftl::data::FrameState<SETTINGS,COUNT>::operator=(ftl::data::FrameState<SETTINGS,COUNT> &&f) {
	pose_ = f.pose_;
	settings_ = f.settings_;
	changed_ = f.changed_;
	config_ = std::move(f.config_);
	f.changed_.clear();
	return *this;
}

#endif  // _FTL_DATA_FRAMESTATE_HPP_
