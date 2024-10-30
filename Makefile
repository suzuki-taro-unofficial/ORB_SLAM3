.PHONY: build
build:
	@if [ ! -f "/.dockerenv" ]; then \
		echo "please run this after make dev"; \
	else \
		./scripts/build.sh; \
	fi

run:
	@if [ ! -f "/.dockerenv" ]; then \
		echo "please run this after make dev"; \
	else \
		./scripts/run.sh; \
	fi

dev:
	@if [ -f "/.dockerenv" ]; then \
		echo "make dev doesn't works inside docker"; \
	else \
		./scripts/dev.sh; \
	fi
