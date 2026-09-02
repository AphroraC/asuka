#include <asuka/utility/load_module.hpp>

#include <dlfcn.h>
#include <spdlog/spdlog.h>

#include <map>
#include <mutex>

namespace asuka {

namespace {

// Process-wide registry of dlopen() handles keyed by library name. Handles
// are intentionally never closed today: plugin code must stay resident for
// the lifetime of the process. The registry is the seam for a future
// unload/reload implementation (see load_module.hpp).
std::mutex handle_mutex;
std::map<std::string, void*> open_handles;

}  // namespace

void* load_so(const std::string& so_name) {
  {
    std::lock_guard<std::mutex> lock(handle_mutex);
    auto itr = open_handles.find(so_name);
    if (itr != open_handles.end()) {
      return itr->second;
    }
  }

  void* handle = dlopen(so_name.c_str(), RTLD_LAZY);
  if (handle == nullptr) {
    spdlog::warn("failed to open {}", so_name);
    spdlog::warn("{}", dlerror());
    return nullptr;
  }

  {
    std::lock_guard<std::mutex> lock(handle_mutex);
    open_handles.emplace(so_name, handle);
  }
  return handle;
}

void* load_symbol(const std::string& so_name, const std::string& symbol_name) {
  void* handle = load_so(so_name);
  if (handle == nullptr) {
    return nullptr;
  }

  auto* func = dlsym(handle, symbol_name.c_str());
  if (func == nullptr) {
    spdlog::warn("failed to find symbol={} in {}", symbol_name, so_name);
    spdlog::warn("{}", dlerror());
  }

  return func;
}

}  // namespace asuka
