# Feetech SDK

`vendor/ftservo/` is an unchanged subset of the official MIT-licensed
[FTServo_Linux](https://github.com/ftservo/FTServo_Linux) SDK at revision
`06fd3356dbd7bccd886b5a70d7ae0fccc6c76d38`.
The original MIT license is retained alongside the sources.

The plugin reuses `SMS_STS` packet construction, register encoding, checksum,
reply ID/length validation and status parsing. Its `CheckedPort` overrides the
virtual serial methods with bounded poll/read/write operations, exclusive port
locking and RAII cleanup. The vendor serial `begin/end` methods are not called.
This avoids upstream serial cleanup and short-write/error handling weaknesses
without modifying the vendored files. No Python SDK process runs in the control loop.
