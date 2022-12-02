/***********************************************************************************/
/* Copyright (C) 2022 eScience and Technologies, Inc.                              */
/*                                                                                 */
/* Licensed under the Apache License, Version 2.0 (the "License"); you may not     */
/* use this file except in compliance with the License. You may obtain a copy of   */
/* the License at                                                                  */
/*                                                                                 */
/* http://www.apache.org/licenses/LICENSE-2.0                                      */
/*                                                                                 */
/* Unless required by applicable law or agreed to in writing, software             */
/* distributed under the License is distributed on an "AS IS" BASIS, WITHOUT       */
/* WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the        */
/* License for the specific language governing permissions and limitations under   */
/* the License.                                                                    */
/*                                                                                 */
/* Developer:  Wayne Staats                                                        */
/*                                                                                 */
/* Method:   main                                                                  */
/* Purpose:  Create the PrepassPlugin and wait for the final decision message.     */
/***********************************************************************************/  


#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include "prepass_plugin/PrepassPlugin.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto sp = std::make_shared<fmcsa_ace_prepass_plugin::PrepassPlugin>();
  sp->initialize();
  rclcpp::spin(sp);
  rclcpp::shutdown();

  return 0;
}
