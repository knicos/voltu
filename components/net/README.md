# FTL Network Library
FTL Net provides an easy to use C++17 network library based around a mix of
streams and RPC. It is now highly optimised to minimise memory copies and
locking, whilst fully taking advantage of all CPU cores for processing
messages received over the network. Each message received is dispatched into
a thread pool. The optimisation works on the basis of a relatively low number
of socket connections but with a high bandwidth and low latency requirement
in sending and receiving on those sockets. Further work would be needed to
be efficient with large or massive numbers of sockets.

The protocol is based on top of [MsgPack](https://github.com/msgpack/msgpack-c)
which works in both C++ and JavaScript. To work with JavaScript the protocol
works over TCP and TCP+Websockets. The library is also cross platform,
supporting Windows and Linux.

It is a template library, allowing simple RPC calls and bindings using the
latest in C++ features such as optionals, lambdas and futures.

## Universe
A [Universe class](cpp/include/ftl/net/universe.hpp) represents a network group
and is the primary means of interaction for the user of the library. It supports
`bind`, `connect`, `call`, `send`, `disconnect`, `asyncCall` and more.

## Peer
A [Peer object](cpp/include/ftl/net/peer.hpp) is a fairly internal object that
wraps a socket connection and deals with all actual sending and receiving over
the network.
