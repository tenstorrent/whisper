# Run every test: make check
#
# Each subdirectory is a test with its own Makefile providing check and
# clean targets. Variables given on the command line, eg. WHISPER or
# TOOLCHAIN, are passed down to them; WHISPER must then be an absolute
# path, as each test runs in its own directory.

TESTS = $(patsubst %/Makefile,%,$(wildcard */Makefile))

.PHONY: check clean $(TESTS)

# Run them all even if one fails, then report the failures.
check:
	@failed=; \
	for t in $(TESTS); do \
	  echo "=== $$t"; \
	  $(MAKE) -C $$t check || failed="$$failed $$t"; \
	done; \
	if [ -n "$$failed" ]; then echo "FAILED:$$failed"; exit 1; fi; \
	echo "All tests passed: $(TESTS)"

$(TESTS):
	$(MAKE) -C $@ check

clean:
	for t in $(TESTS); do $(MAKE) -C $$t clean; done
