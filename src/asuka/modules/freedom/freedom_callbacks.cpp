#include <asuka/modules/freedom/freedom_callbacks.hpp>

#include <asuka/modules/freedom/freedom_types.hpp>

namespace asuka {

CallbackSlot<void(const std::shared_ptr<const FreedomScanRemoval>&)> FreedomCallbacks::on_freedom_scan_removal;
CallbackSlot<void(const std::shared_ptr<const FreedomRaycastEnhancement>&)> FreedomCallbacks::on_freedom_raycast_enhancement;
CallbackSlot<void(const std::shared_ptr<const FreedomMapRemoval>&)> FreedomCallbacks::on_freedom_map_removal;

}  // namespace asuka
