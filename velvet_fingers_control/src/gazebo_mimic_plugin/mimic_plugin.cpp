/*
 * Copyright (c) 2014, ISR, University of Coimbra
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Goncalo Cabrita
 * Gazebo plugin for the FSR Husky Mine Detection Arm with th goal of solving
 * the <mimic> joint tag limitations in Gazebo
 */

#include <gazebo_mimic_plugin/mimic_plugin.h>

using namespace gazebo;

MimicPlugin::MimicPlugin()
{
  kill_sim = false;

  joint_.reset();
  mimic_joint_.reset();

  link_.reset();
  mimic_link_.reset();
  parent_link_.reset();
}

MimicPlugin::~MimicPlugin()
{
  //event::Events::DisconnectWorldUpdateBegin(this->updateConnection);

  kill_sim = true;
}

void MimicPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  this->model_ = _parent;
  this->world_ = this->model_->GetWorld();


  joint_name_ = "joint";
  if (_sdf->HasElement("joint")){
    joint_name_ = _sdf->GetElement("joint")->Get<std::string>();
  }
  mimic_joint_name_ = "mimicJoint";
  if (_sdf->HasElement("mimicJoint")){
    mimic_joint_name_ = _sdf->GetElement("mimicJoint")->Get<std::string>();
  }
  multiplier_ = 1.0;
  if (_sdf->HasElement("multiplier")){
    multiplier_ = _sdf->GetElement("multiplier")->Get<double>();
  }

  // Get the name of the parent model
  std::string modelName = _sdf->GetParent()->Get<std::string>("name");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&MimicPlugin::UpdateChild, this));
  gzdbg << "Plugin model name: " << modelName << "\n";

  joint_ = model_->GetJoint(joint_name_);
  mimic_joint_ = model_->GetJoint(mimic_joint_name_);

  //反力伝達用
  link_name_ = "link";
  if (_sdf->HasElement("link")){
    link_name_ = _sdf->GetElement("link")->Get<std::string>();
  }
  mimic_link_name_ = "mimicLink";
  if (_sdf->HasElement("mimicLink")){
    mimic_link_name_ = _sdf->GetElement("mimicLink")->Get<std::string>();
  }
  parent_link_name_ = "parentLink";
  if (_sdf->HasElement("parentLink")){
    parent_link_name_ = _sdf->GetElement("parentLink")->Get<std::string>();
  }
  link_ = model_->GetLink(link_name_);
  mimic_link_ = model_->GetLink(mimic_link_name_);
  parent_link_ = model_->GetLink(parent_link_name_);
}

void MimicPlugin::UpdateChild()
{
	//仕組み：従属関節を、位置入力・力出力の系として受動的に運動させることで平行リンクを再現する
	//①従属関節の角度は、強制的に基準関節の角度で上書きする(関節速度も計算に使用しているらしいので上書きする)
	//②従属対偶に外部から加わる力・トルクを、従属関節周りの力・トルクに変換→従属関節周りのトルクのみ基準関節周りのトルクに伝達
	//※従属リンクの摩擦は0にしないとうまく動かない。応答が1ステップ遅れてしまうのが原因と思われる

	//ここから①
	ignition::math::Vector3d joint_position(joint_->Position(0),joint_->Position(1),joint_->Position(2));
	ignition::math::Vector3d joint_velocity(joint_->GetVelocity(0),joint_->GetVelocity(1),joint_->GetVelocity(2));
	ignition::math::Vector3d joint_force(joint_->GetForce(0),joint_->GetForce(1),joint_->GetForce(2));
	ignition::math::Vector3d mimic_force;
	ignition::math::Vector3d mimic_torque;
	mimic_force = mimic_link_->RelativeForce();
	mimic_torque = mimic_link_->RelativeTorque();

	mimic_joint_ -> SetPosition(0, joint_position.X());
	mimic_joint_ -> SetPosition(1, joint_position.Y());
	mimic_joint_ -> SetPosition(2, joint_position.Z());
	mimic_joint_ -> SetVelocity(0,joint_velocity.X());
	mimic_joint_ -> SetVelocity(1,joint_velocity.Y());
	mimic_joint_ -> SetVelocity(2,joint_velocity.Z());
	mimic_joint_ -> SetForce(0,0);
	mimic_joint_ -> SetForce(1,0);
	mimic_joint_ -> SetForce(2,0);

	//ここから②
	//Relativeがつく場合、リンクの原点を基準とする力・トルクになる(例：link_-> AddRelativeForce(mimic_link_->RelativeForce());)
	//Worldがつく場合、ワールド座標系にを基準とする力・トルクになる(例：link_-> AddForce(mimic_link_->WorldForce());)
	//リンク原点が親とのジョイント位置に一致すると仮定した
	//挙動を見る限り、すでに原点周りのトルクと、原点を並進で動かそうとする力に分解されている
	//従属リンク(mimic_link_)について、原点並進力はそのまま、原点周りのトルクのみ基準リンク(link_)に伝達する(mimic_linkのトルクはなくなったとみなす)
//	link_-> AddForce(mimic_link_->WorldForce());
//	link_-> AddTorque(mimic_link_->WorldTorque());
//	link_-> AddRelativeForce(mimic_force);
//	mimic_torque.X(0);
//	mimic_torque.Z(0);
	link_-> AddRelativeTorque(mimic_torque);
//	link_-> AddRelativeForce(mimic_link_->RelativeForce());
//	link_-> AddRelativeTorque(mimic_link_->RelativeTorque());
//	parent_link_-> AddRelativeForce(-1.0*mimic_force);
//	parent_link_-> AddRelativeTorque(-1.0*mimic_torque);
//	mimic_link_-> AddRelativeForce(-1.0*mimic_force);
//	mimic_link_-> AddRelativeTorque(-1.0*mimic_torque);
	ignition::math::Vector3d ZeroForce(0,0,0);
	mimic_link_-> SetTorque(ZeroForce);
	mimic_link_-> SetForce(ZeroForce);
	mimic_link_-> AddRelativeForce(mimic_force);
//	mimic_link_-> AddRelativeTorque(-1.0*mimic_torque);
//	parent_link_-> AddForce(mimic_link_->WorldForce());
//	parent_link_-> AddForce(-1.0*mimic_link_->WorldForce());
//	parent_link_-> AddTorque(-1.0*mimic_link_->WorldTorque());

}

GZ_REGISTER_MODEL_PLUGIN(MimicPlugin);
