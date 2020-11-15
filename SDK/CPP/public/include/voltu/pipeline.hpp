/**
 * @file pipeline.hpp
 * @copyright Copyright (c) 2020 Nicolas Pope, MIT License
 * @author Nicolas Pope
 */

#pragma once

#include <voltu/defines.hpp>
#include <voltu/types/frame.hpp>
#include <voltu/operator.hpp>
#include <memory>

namespace voltu
{

/**
 * @brief Manage a pipeline of frame procesing operations.
 * 
 * A frame processing pipeline can be constructed from a range of different
 * operators. One example is the fusion operator that merges all input data
 * together with a model. The pipeline of operators runs asynchonously but in
 * sequential order and modifies the frames internal data.
 * 
 * @see voltu::Frame
 * @see voltu::Operator
 */
class Pipeline
{
public:
	virtual ~Pipeline() = default;
	
	/**
	 * @brief Send a frame for processing.
	 * 
	 * This method is non-blocking and will therefore return before operator
	 * processing has been completed. The data inside the frame object is
	 * modified during this process, so the data should not be accessed until
	 * the pipeline is completed.
	 * 
	 * @note
	 * You must not submit multiple frames to the same pipeline without
	 * ensuring completion. You must also not access frame data during
	 * processing. The room object associated with the frame must remain valid
	 * for the duration of processing.
	 * 
	 * @param frame A room data frame.
	 * 
	 * @throw voltu::exceptions::InvalidFrameObject If the frame is invalid
	 * @throw voltu::exceptions::IncompatibleOperation If the pipeline given
	 *     cannot be applied to the frame for some reason.
	 */
	PY_API virtual void submit(const voltu::FramePtr &frame) = 0;

	/**
	 * @brief Block until all processing is completed.
	 * 
	 * If a frame has been submitted, this can be used to block until all
	 * processing has finished. If no frame has been submitted then this
	 * method will currently block until timeout.
	 * 
	 * @todo Allow timeout of -1 and exception on no frame.
	 * 
	 * @param timeout Millisecond timeout, or 0 for non-blocking check.
	 * @return True if completed
	 */
	PY_API virtual bool waitCompletion(int timeout, bool except=false) = 0;

	/**
	 * @brief Add an operator to this pipeline.
	 * 
	 * Each operator is appended to the pipeline in the order added, and
	 * therefore will be processed sequentially in that same order. One type
	 * of operator can be appended multiple times, useful if different settings
	 * are to be used each time. Settings can be accessed from the returned
	 * operator management instance.
	 * 
	 * @see voltu::Operator
	 * 
	 * @throw voltu::exceptions::BadParameterValue For invalid operators
	 * @throw voltu::exceptions::NotImplemented If operator not implemented for SDK
	 * @return Operator management instance for settings.
	 */
	PY_API virtual voltu::OperatorPtr appendOperator(voltu::OperatorId id) = 0;
};

typedef std::shared_ptr<Pipeline> PipelinePtr;

}
