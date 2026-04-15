/*******************************************************************************
 *  Copyright (c) 2020 robomaster-oss, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/
#include <mutex>
#include <map>
#include <gz/common/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Material.hh>

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Conversions.hh>

#include "LightBarController.hh"

using namespace gz;
using namespace sim;
using namespace systems;


sdf::Material GetMaterial(int state){
    sdf::Material m;
    gz::math::Color color;
    gz::math::Color emissiveColor;

    if(state == 0){
        color.Set(1, 1, 1, 1);
        emissiveColor.Set(0.5, 0.5, 0.5, 1);
    }else if(state == 1){
        color.Set(1, 0, 0, 1);
        emissiveColor.Set(1, 0.2, 0.2, 1);
    }else if(state == 2){
        color.Set(0, 0, 1, 1);
        emissiveColor.Set(0.2, 0.2, 1, 1);
    }else if(state == 3){
        color.Set(1, 1, 0, 1);
        emissiveColor.Set(1, 1, 0.2, 1);
    }else{
        color.Set(1, 1, 1, 1);
        emissiveColor.Set(1, 1, 1, 1);
    }

    m.SetAmbient(color);
    m.SetDiffuse(color);
    m.SetSpecular(color);
    m.SetEmissive(emissiveColor);

    return m;
}


struct VisualEntityInfo {
    Entity entity;
    VisualEntityInfo(Entity _entity)
        : entity(_entity)
    {
    }
};

class gz::sim::systems::LightBarControllerPrivate
{
public:
    void OnCmd(const gz::msgs::Int32 &_msg);
    void Init(gz::sim::EntityComponentManager &_ecm);
    void ApplyMaterial(gz::sim::EntityComponentManager &_ecm,
                       const sdf::Material &_material);

public:
    transport::Node node;
    Model model{kNullEntity};
    sdf::Model modelSdf;
    std::vector<std::string> linkVisuals;
    std::vector<VisualEntityInfo> visualEntityInfos;
    bool isInit{false};
    // 0:no light, 1:red light, 2:blue light, 3:yellow light, 4:white light
    int targetState{0};
    bool change{false};
    std::mutex targetMutex;
};

/******************implementation for LightBarController************************/
LightBarController::LightBarController() : dataPtr(std::make_unique<LightBarControllerPrivate>())
{
}

void LightBarController::Configure(const Entity &_entity,
                             const std::shared_ptr<const sdf::Element> &_sdf,
                             EntityComponentManager &_ecm,
                             EventManager &)
{
    this->dataPtr->model = Model(_entity);
    if (!this->dataPtr->model.Valid(_ecm))
    {
        gzerr << "LightBarController plugin should be attached to a model entity. Failed to initialize." << std::endl;
        return;
    }
    // Get params from SDF
    std::string controller_name = "light_bar_controller";
    if (_sdf->HasElement("controller_name"))
    {
        controller_name = _sdf->Get<std::string>("controller_name");
    }
    
    if (_sdf->HasElement("initial_color"))
    {
        std::map<std::string,int> color_map{{"none",0},{"red",1},{"blue",2},{"yellow",3},{"white",4}};
        auto color = _sdf->Get<std::string>("initial_color");
        if(color_map.find(color)!=color_map.end()){
            this->dataPtr->targetState = color_map[color];
            this->dataPtr->change = true;
        }else{
            gzwarn << "LightBarController color [" << color << "] is invalid." << std::endl;
        }
    }
    // link_visual
    auto ptr = const_cast<sdf::Element *>(_sdf.get());
    sdf::ElementPtr sdfElem = ptr->GetElement("link_visual");
    while (sdfElem)
    {
        auto path = sdfElem->Get<std::string>();
        this->dataPtr->linkVisuals.push_back(std::move(path));
        sdfElem = sdfElem->GetNextElement("link_visual");
    }
    // Model Sdf
    this->dataPtr->modelSdf = _ecm.Component<components::ModelSdf>(_entity)->Data();
    // Subscribe to commands
    std::string topic{this->dataPtr->model.Name(_ecm) +"/"+controller_name+ "/set_state"};
    this->dataPtr->node.Subscribe(topic, &LightBarControllerPrivate::OnCmd, this->dataPtr.get());
    gzmsg << "LightBarController subscribing to int32 messages on [" << topic << "]" << std::endl;
}

void LightBarController::PreUpdate(const gz::sim::UpdateInfo &_info,
                             gz::sim::EntityComponentManager &_ecm)
{
    if(_info.paused){
        return;
    }
    if(!this->dataPtr->isInit){
        this->dataPtr->Init(_ecm);
        this->dataPtr->isInit = true;
    }
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->targetMutex);
        if(this->dataPtr->change){
            auto targetMaterial = GetMaterial(this->dataPtr->targetState);
            this->dataPtr->ApplyMaterial(_ecm, targetMaterial);
            this->dataPtr->change = false;
        }
    }
}


/******************implementation for LightBarControllerPrivate******************/
void LightBarControllerPrivate::OnCmd(const gz::msgs::Int32 &_msg)
{
    std::lock_guard<std::mutex> lock(this->targetMutex);
    this->targetState = _msg.data();
    this->change = true;
}

void LightBarControllerPrivate::Init(gz::sim::EntityComponentManager &_ecm){
    for(const auto &linkVisual : this->linkVisuals){
        auto v = common::split(linkVisual,"/");
        if(v.size() == 2){
            auto link = this->model.LinkByName(_ecm, v[0]);
            auto visual = _ecm.EntityByComponents(
                components::ParentEntity(link),
                components::Name(v[1]),
                components::Visual());
            if(visual != kNullEntity){
                this->visualEntityInfos.emplace_back(visual);
                continue;
            }
        }
        gzerr << "LightBarController: visual element of link ["
               << linkVisual << "] is invalid" << std::endl;
    }
}

void LightBarControllerPrivate::ApplyMaterial(
    gz::sim::EntityComponentManager &_ecm,
    const sdf::Material &_material)
{
    for(auto &info : this->visualEntityInfos){
        auto materialComp = _ecm.Component<components::Material>(info.entity);
        if(materialComp){
            *materialComp = components::Material(_material);
            _ecm.SetChanged(info.entity, components::Material::typeId,
                            ComponentState::OneTimeChange);
        }else{
            _ecm.CreateComponent(info.entity, components::Material(_material));
        }
    }
}

/******************register*************************************************/
GZ_ADD_PLUGIN(LightBarController,
                    gz::sim::System,
                    LightBarController::ISystemConfigure,
                    LightBarController::ISystemPreUpdate
                    )

GZ_ADD_PLUGIN_ALIAS(LightBarController, "gz::sim::systems::LightBarController")
