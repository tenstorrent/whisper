# Console I/O

This example shows two ways of doing console I/O with Whisper:

- `print_htif`: this subroutine uses the HTIF interface which was introduced in
  the previous lesson.
- `print_whisper`: this uses the `__whisper_console_io` symbol, which is
  simpler and less general that HTIF because it can only be used for
  reading/writine the console.
