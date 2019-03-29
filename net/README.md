# FTL Network Library
FTL Net provides stream, RPC and Peer-2-Peer functionality for the FTL system.
The idea is to allow an efficient mapping to operating system sockets to
minimise userspace copy operations, whilst still allowing for data packing for
smaller RPC calls. The P2P component implements different rpc search strategies
to allow calls to find one, all, many or specific results across the network.

Multiple protocols are supported, and it is intended that NAT traversal will be
included. However, security, whether encryption or identification, is not
considered presently.


