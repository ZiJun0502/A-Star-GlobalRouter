CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wno-unused-variable -Wno-unused-parameter -g -Ofast -fopenmp -MMD -MP -I/usr/include/boost
# LDFLAGS = -lboost_system -lboost_filesystem

# Folders
SRCDIR = src
BUILDDIR = build

# Find all .cpp files in src/ directory
SRCS = $(wildcard $(SRCDIR)/*.cpp)
OBJS = $(patsubst $(SRCDIR)/%.cpp,$(BUILDDIR)/%.o,$(SRCS))
DEPS = $(OBJS:.o=.d)

# Target binary in the root directory
TARGET = D2DGRter

all: $(TARGET)

# Link all object files to create the final binary
$(TARGET): $(OBJS)
	$(CXX) -g -o $(TARGET) $(OBJS) 

# Compile each .cpp file to the corresponding .o file in the build/ directory
$(BUILDDIR)/%.o: $(SRCDIR)/%.cpp
	@mkdir -p $(BUILDDIR)
	$(CXX) $(CXXFLAGS) -I$(SRCDIR) -c $< -o $@

# Include dependency files for header tracking
-include $(DEPS)

clean:
	rm -f $(BUILDDIR)/*.o $(BUILDDIR)/*.d $(TARGET)
