#pragma once

#include <memory>
#include <string>

namespace asuka {

// Returns the dlopen() handle for so_name, opening the library if it is not
// already loaded. Handles live in a process-wide registry owned by asuka_core
// so that a future unload/reload implementation can close them again.
//
// Unload order of operations (not implemented yet):
// 1. stop every thread that can emit callback slots (AsukaROS teardown order),
// 2. destroy the modules so they unsubscribe from the callback slots,
// 3. take the handle from the registry, dlclose() it and drop the entry.
void* load_so(const std::string& so_name);

void* load_symbol(const std::string& so_name, const std::string& symbol_name);

template <typename Module>
std::shared_ptr<Module> load_module_from_so(const std::string& so_name, const std::string& func_name) {
  auto func = (Module * (*)()) load_symbol(so_name, func_name);
  if (func == nullptr) {
    return nullptr;
  }

  return std::shared_ptr<Module>(func());
}

}  // namespace asuka
