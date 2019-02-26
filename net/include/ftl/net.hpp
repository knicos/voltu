#ifndef _FTL_NET_HPP_
#define _FTL_NET_HPP_

#include <memory>
#include <functional>

namespace ftl {
namespace net {

class Listener;
class Socket;

const int MAX_CONNECTIONS = 100; // TODO Is this a good number?

/**
 * Start a listening socket for new connections on the given URI. An example
 * URI might be:
 *     tcp://localhost:9000.
 */
std::shared_ptr<Listener> listen(const char *uri);

/**
 * Accepts tcp, ipc and ws URIs. An example would be:
 *  ws://ftl.utu.fi/api/connect
 */
std::shared_ptr<Socket> connect(const char *uri);

/**
 * Start a loop to continually check for network messages. If the async
 * parameter is false then this function will block as long as any connections
 * or listeners remain active.
 *
 * @param async Use a separate thread.
 */
bool run(bool async=false);

/**
 * Wait for a bunch of messages, but return once at least one has been
 * processed.
 */
bool wait();

void wait(std::function<bool(void)>, float t=3.0f);

/**
 * Check and process any waiting messages, but do not block if there are none.
 */
bool check();

/**
 * Ensure that the network loop terminates, whether a separate thread or not.
 */
void stop();

/**
 * Is the network loop running in another thread?
 */
bool is_async();

/**
 * Is the network loop currently handling a message?
 */
bool is_handling();

}
}

#endif // _FTL_NET_HPP_
