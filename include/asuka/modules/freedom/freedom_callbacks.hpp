#pragma once

#include <memory>

#include <asuka/utility/callback_slot.hpp>

namespace asuka {

// Freedom payload types are the freedom extension's own (freedom_types.hpp);
// consumers of the slots see them through the spelled shared_ptr, identical
// to FreedomXxx::ConstPtr.
struct FreedomScanRemoval;
struct FreedomRaycastEnhancement;
struct FreedomMapRemoval;

/**
 * @brief The freedom extension's own callback slots, hosted by
 *        libasuka_freedom.so (definitions in freedom_callbacks.cpp).
 *
 * Deliberately NOT in the core asuka::Callbacks: a point-to-point extension
 * stream must not occupy the global bus or grow the core's compile-time slot
 * set. Third-party extensions can follow the same pattern (own header plus
 * one defining .cpp in their own .so) without touching asuka_core.
 *
 * Single instance across .so files relies on the same mechanism as the core
 * slots: the static objects are defined in exactly one shared library, and
 * every consumer links it -- the dynamic loader's DT_NEEDED soname dedup
 * collapses repeat loads into the first instance. (Defining them inline in
 * this header would give every .so its own copy and silently break delivery,
 * exactly like the core slots.)
 *
 * Emission contract (mirrors the core slots'): fired synchronously by the
 * freedom extension on its own processing thread, once per integrate cycle
 * at the matching pipeline stage; payloads are freshly allocated per
 * emission and never mutated afterwards; null fields mark visualization
 * streams disabled in the freedom config. Observers must not block
 * (freedom_viewer enqueues and publishes on its own thread).
 */
struct FreedomCallbacks {
  static CallbackSlot<void(const std::shared_ptr<const FreedomScanRemoval>&)> on_freedom_scan_removal;
  static CallbackSlot<void(const std::shared_ptr<const FreedomRaycastEnhancement>&)> on_freedom_raycast_enhancement;
  static CallbackSlot<void(const std::shared_ptr<const FreedomMapRemoval>&)> on_freedom_map_removal;
};

}  // namespace asuka
