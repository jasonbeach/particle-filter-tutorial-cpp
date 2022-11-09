from conans import ConanFile, CMake, tools

# version MUST be supplied on command line


class ParticleFilterConan(ConanFile):
  name = "particle_filter"
  scm = {
    "type": "git",
    "subfolder": "particle_filter",
    "url": "ssh://git@coca.ssci.com:7999/rapcore/particle_filter.git",
    "revision": "auto"
  }
  license = "Proprietary"
  author = "Jason Beach <jason.m.beach@gmail.com>"
  description = "Particle Filter Tutorial"
  topics = ("raptor")
  settings = "os", "compiler", "build_type", "arch"
  options = {"shared": [True, False], "fPIC": [True, False]}
  default_options = {"shared": True, "fPIC": True}
  generators = "cmake_find_package", "cmake_paths"
  _cmake = None

  def configure(self):
    if self.settings.compiler.libcxx == "libstdc++":
      raise Exception("This package is only compatible with libstdc++11")

  def config_options(self):
    if self.settings.os == "Windows":
      del self.options.fPIC

  def build(self):
    if self._cmake:
      return self._cmake
    self._cmake = CMake(self, generator="Ninja")
    self._cmake.configure(source_folder="particle_filter",
                          defs={"CMAKE_TOOLCHAIN_FILE": "conan_paths.cmake"})
    
    self._cmake.build()

  def requirements(self):
    self.requires('catch2/2.13.4')
    self.requires('fmt/9.1.0')
    self.requires('eigen/3.4.0')
    
  def package(self):
    self._cmake.install()

  def package_info(self):
    self.cpp_info.libs = [""]
