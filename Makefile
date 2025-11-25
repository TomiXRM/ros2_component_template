.PHONY: help format check-format build clean test

# Default target
help:
	@echo "Available targets:"
	@echo "  make build         - Build all packages with symlink install"
	@echo "  make format        - Format all C++ files using clang-format"
	@echo "  make check-format  - Check C++ formatting without modifying files"
	@echo "  make clean         - Remove build artifacts (build/, install/, log/)"
	@echo "  make test          - Run all tests"
	@echo "  make help          - Show this help message"

# Build all packages with symlink install
build:
	colcon build --symlink-install

# Format all C++ source files
format:
	ament_clang_format --config .clang-format --reformat src/

# Check formatting without modifying files
check-format:
	ament_clang_format --config .clang-format src/

# Clean build artifacts
clean:
	rm -rf build/ install/ log/

# Run tests
test:
	colcon test
	colcon test-result --verbose
