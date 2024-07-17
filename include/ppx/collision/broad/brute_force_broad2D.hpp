#pragma once

#include "ppx/collision/broad/broad_phase2D.hpp"
#include "ppx/collider/collider_manager2D.hpp"

namespace ppx
{
class brute_force_broad2D final : public broad_phase2D
{
  public:
    using broad_phase2D::broad_phase2D;

    const char *name() const override;

  private:
    void find_new_pairs(const std::vector<collider2D *> &to_update) override;

    void update_pairs_st(const std::vector<collider2D *> &to_update);
    void update_pairs_mt(const std::vector<collider2D *> &to_update);
};
} // namespace ppx
