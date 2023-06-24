/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 * @brief Defines the Factory class.
 */

#pragma once

#include <map>
#include <memory>
#include <utility>

#include "cyber/common/macros.h"

#include "cyber/common/log.h"

/**
 * @namespace apollo::common::util
 * @brief apollo::common::util
 */
namespace apollo {
namespace common {
namespace util {

/**
 * @class Factory
 * @brief Implements a Factory design pattern with Register and Create methods
 *
 * The objects created by this factory all implement the same interface
 * (namely, AbstractProduct). This design pattern is useful in settings where
 * multiple implementations of an interface are available, and one wishes to
 * defer the choice of the implementation in use.
 *
 * @param IdentifierType Type used for identifying the registered classes,
 * typically std::string.
 * @param AbstractProduct The interface implemented by the registered classes
 * @param ProductCreator Function returning a pointer to an instance of
 * the registered class
 * @param MapContainer Internal implementation of the function mapping
 * IdentifierType to ProductCreator, by default std::unordered_map
 */
template <typename IdentifierType, class AbstractProduct,//IdentifierType和AbstractProduct创建对象时需要传入类型
          class ProductCreator = AbstractProduct *(*)(), //类ProductCreator是AbstractProduct的衍生
          class MapContainer = std::map<IdentifierType, ProductCreator>>//类MapContainer是IdentifierType和ProductCreator的衍生，因此不要特意传入来指定类型
class Factory {
 public:
  /**
   * @brief Registers the class given by the creator function, linking it to id.
   * Registration must happen prior to calling CreateObject.
   * @param id Identifier of the class being registered
   * @param creator Function returning a pointer to an instance of
   * the registered class
   * @return True if the key id is still available
   */
  bool Register(const IdentifierType &id, ProductCreator creator) {
    return producers_.insert(std::make_pair(id, creator)).second;
  }

  bool Contains(const IdentifierType &id) {
    return producers_.find(id) != producers_.end();
  }

  /**
   * @brief Unregisters the class with the given identifier
   * @param id The identifier of the class to be unregistered
   */
  bool Unregister(const IdentifierType &id) {
    return producers_.erase(id) == 1;
  }

  void Clear() { producers_.clear(); }

  bool Empty() const { return producers_.empty(); }

  /**
   * @brief Creates and transfers membership of an object of type matching id.
   * Need to register id before CreateObject is called. May return nullptr
   * silently.
   * @param id The identifier of the class we which to instantiate
   * @param args the object construction arguments
   */
  template <typename... Args>
  std::unique_ptr<AbstractProduct> CreateObjectOrNull(const IdentifierType &id,
                                                      Args &&... args) {
    auto id_iter = producers_.find(id);//producers_中存放了所注册控制器的<id,指向控制器的指针>，在producers_中寻找名字为id的迭代器，并返回迭代器
    if (id_iter != producers_.end()) {//如果遍历每结束
      return std::unique_ptr<AbstractProduct>(
          (id_iter->second)(std::forward<Args>(args)...));//将指向所注册的控制器指针变为智能指针，并返回
    }
    return nullptr;
  }

  /**
   * @brief Creates and transfers membership of an object of type matching id.
   * Need to register id before CreateObject is called.
   * @param id The identifier of the class we which to instantiate
   * @param args the object construction arguments
   */
  template <typename... Args>
  std::unique_ptr<AbstractProduct> CreateObject(const IdentifierType &id,
                                                Args &&... args) {
    auto result = CreateObjectOrNull(id, std::forward<Args>(args)...);
    AERROR_IF(!result) << "Factory could not create Object of type : " << id;
    return result;
  }

 private:
  MapContainer producers_;
};

}  // namespace util
}  // namespace common
}  // namespace apollo
