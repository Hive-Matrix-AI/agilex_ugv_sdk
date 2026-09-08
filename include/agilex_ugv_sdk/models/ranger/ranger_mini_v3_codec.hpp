// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "agilex_ugv_sdk/models/ranger/ranger_mini_v3_types.hpp"
#include "agilex_ugv_sdk/protocol/protocol_codec.hpp"

namespace agilex::ugv {

class RangerMiniV3Codec final : public ProtocolCodec {
 public:
  [[nodiscard]] ModelCapabilities capabilities() const noexcept override;
  [[nodiscard]] EncodeResult encode(const Command& command) override;
  [[nodiscard]] DecodeResult decode(const CanFrame& frame) const override;
};

[[nodiscard]] ProtocolCodecPtr make_ranger_mini_v3_codec();

}  // namespace agilex::ugv
