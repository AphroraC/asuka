#include <asuka/modules/freedom/freedom_module.hpp>

extern "C" asuka::ExtensionModule* create_extension_module() {
  return new asuka::FreedomModule();
}
