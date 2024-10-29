.PHONY: build
build:
	@if [ ! -f "/.dockerenv" ]; then \
		echo "please run this after make dev"; \
		exit 1; \
	fi
	./scripts/build.sh

build_ros:
	@if [ ! -f "/.dockerenv" ]; then \
		echo "please run this after make dev"; \
		exit 1; \
	fi
	./scripts/build_ros.sh

run:
	@if [ ! -f "/.dockerenv" ]; then \
		echo "please run this after make dev"; \
		exit 1; \
	fi
	./scripts/run.sh

dev:
	./scripts/dev.sh
