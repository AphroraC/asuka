#include <asuka/utility/extension_module.hpp>

#include <asuka/utility/load_module.hpp>

namespace asuka {

std::shared_ptr<ExtensionModule> ExtensionModule::load_module(const std::string& so_name) {
  return load_module_from_so<ExtensionModule>(so_name, "create_extension_module");
}

}  // namespace asuka
