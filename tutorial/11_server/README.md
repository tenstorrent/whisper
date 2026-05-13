# Server Mode

This example includes a very basic client program which connects to Whisper
over TCP and drives a few commands to it.

Whisper also allows for clients to communicate over shared memory using the
`--shm` option but this requires using different APIs than the socket version.
