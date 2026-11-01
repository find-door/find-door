// Ceres 솔버에 그대로 위임하면서 최적화 호출과 직전 연결을 JSONL로 기록한다.
// slam_toolbox 2.6.10에서 Compute()는 TryCloseLoop의 수락 경로에서만 호출된다.
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "karto_sdk/Mapper.h"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mapping_slam_debug
{

namespace
{
std::string pose_json(const karto::Pose2 & pose)
{
  std::ostringstream out;
  out << std::setprecision(9) << '[' << pose.GetX() << ',' << pose.GetY() << ',' <<
    pose.GetHeading() << ']';
  return out.str();
}

std::string scan_json(karto::LocalizedRangeScan * scan)
{
  std::ostringstream out;
  out << std::setprecision(15) << "{\"id\":" << scan->GetUniqueId() <<
    ",\"time\":" << scan->GetTime() <<
    ",\"odom\":" << pose_json(scan->GetOdometricPose()) <<
    ",\"corrected\":" << pose_json(scan->GetCorrectedPose()) << '}';
  return out.str();
}
}  // namespace

class LoggingSolver : public karto::ScanSolver
{
public:
  LoggingSolver()
  : loader_("slam_toolbox", "karto::ScanSolver") {}

  void Configure(rclcpp::Node::SharedPtr node) override
  {
    const auto path = node->declare_parameter("loop_log_path", std::string());
    if (path.empty()) {
      throw std::runtime_error("loop_log_path가 필요합니다");
    }
    log_.open(path, std::ios::app);
    if (!log_) {
      throw std::runtime_error("기록 파일을 열 수 없습니다: " + path);
    }
    inner_ = loader_.createSharedInstance("solver_plugins::CeresSolver");
    inner_->Configure(node);
  }

  void Compute() override
  {
    std::ostringstream out;
    out << "{\"seq\":" << sequence_++ << ",\"pending\":[";
    for (size_t i = 0; i < pending_.size(); ++i) {
      out << (i ? "," : "") << pending_[i];
    }
    pending_.clear();
    inner_->Compute();
    out << "],\"correction_count\":" << inner_->GetCorrections().size() << "}\n";
    log_ << out.str() << std::flush;
  }

  const IdPoseVector & GetCorrections() const override {return inner_->GetCorrections();}

  void AddNode(karto::Vertex<karto::LocalizedRangeScan> * vertex) override
  {
    // 이전 스캔의 일반 연결이 다음 루프 기록에 섞이지 않게 비운다.
    pending_.clear();
    inner_->AddNode(vertex);
  }

  void AddConstraint(karto::Edge<karto::LocalizedRangeScan> * edge) override
  {
    auto * link = static_cast<karto::LinkInfo *>(edge->GetLabel());
    const auto covariance = link->GetCovariance();
    std::ostringstream out;
    out << std::setprecision(9) << "{\"source\":" << scan_json(edge->GetSource()->GetObject()) <<
      ",\"target\":" << scan_json(edge->GetTarget()->GetObject()) <<
      ",\"link\":" << pose_json(link->GetPoseDifference()) <<
      ",\"variance\":[" << covariance(0, 0) << ',' << covariance(1, 1) << ',' <<
      covariance(2, 2) << "]}";
    pending_.push_back(out.str());
    inner_->AddConstraint(edge);
  }

  void RemoveNode(kt_int32s id) override {inner_->RemoveNode(id);}
  void RemoveConstraint(kt_int32s source, kt_int32s target) override
  {
    inner_->RemoveConstraint(source, target);
  }
  void Clear() override {inner_->Clear();}
  void Reset() override {inner_->Reset();}
  std::unordered_map<int, Eigen::Vector3d> * getGraph() override {return inner_->getGraph();}
  void ModifyNode(const int & id, Eigen::Vector3d pose) override {inner_->ModifyNode(id, pose);}
  void GetNodeOrientation(const int & id, double & pose) override
  {
    inner_->GetNodeOrientation(id, pose);
  }

private:
  pluginlib::ClassLoader<karto::ScanSolver> loader_;
  std::shared_ptr<karto::ScanSolver> inner_;
  std::vector<std::string> pending_;
  std::ofstream log_;
  long sequence_ = 0;
};

}  // namespace mapping_slam_debug

PLUGINLIB_EXPORT_CLASS(mapping_slam_debug::LoggingSolver, karto::ScanSolver)
