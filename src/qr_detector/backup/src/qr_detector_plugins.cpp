#include "qr_detector/scan_qr_code.hpp"
#include <behaviortree_cpp_v3/bt_factory.h>

namespace qr_detector {

void RegisterQRDetector(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<ScanQRCode>("ScanQRCode");
}

}  // namespace qr_detector
