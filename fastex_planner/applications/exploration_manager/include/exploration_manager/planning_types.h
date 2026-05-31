#ifndef _PLANNING_TYPES_H_
#define _PLANNING_TYPES_H_

#include <cstdint>

namespace fastex_explorer
{
/**
 * @brief Unified result code used by exploration planning submodules.
 */
enum class PLAN_RESULT : int8_t
{
    /// No valid frontier or planning target is available.
    NO_FRONTIER,
    /// Planning failed due to search, optimization, or data issues.
    FAIL,
    /// Planning completed successfully.
    SUCCEED
};
} // namespace fastex_explorer

#endif // _PLANNING_TYPES_H_
