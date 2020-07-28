GUI

Nanogui based graphical user interface.

General:
 * Do not modify gui outside gui thread (main). Modifications must be done in
   GUI callbacks or draw().
 * Expensive processing should be moved out of gui thread (draw() and callbacks)
 * Module is only required to implement Module. Each module is expected to be
   loaded only once.

Classes

Screen
 * Implements main screen: toolbar and view
 * Interface for registering new modules.
 * Interface for adding/removing buttons
 * Interface for setting active View. Inactive view is removed and destroyed if
   no other references are remaining.
 * Note: toolbar could be a module, but other modules likely assume it is
   always available anyways.
 * Centralized access to Nanogui::Themes and custom non-theme colors.

Module (controller)
 * GUI module class wraps pointers for io, config and net. Initialization should
   add necessary buttons to Screen
 * Build necessary callbacks to process data from InputOutput to view.
   Note: If callback passes data to view, callback handle should be owned by
   the view or Module has to keep a nanogui reference to the View. Also note
   that View destructor is called when active view is replaced.

View
 * Active view will be the main window; only one view can be active at time
 * Button callbacks (eg. those registered by module init) may change active view
 * Destroyed when view is changed. Object lifetime can be used to remove
   callbacks from InputOutput (TODO: only one active callback supported at the
   moment)
 * Implementations do not have to inherit from View. Popup/Window/Widget... can
   be used to implement UI components available from any mode (config, record).
 * Receives all unprocessed keyboard events.

InputOutput
 * Contains pointers to all required FTL objects (network/rendering/feed/...).
 * Speaker

NanoGUI notes:
 * If disposing Window in widget destructor, window->parent() reference count
   must be checked and dispose() only called if refCount > 0. (segfault at exit)
 * Nanogui does not dispose popup windows automatically. See above point if
   using destructor for clean up.
