
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
/* Purpose:  Create the CellularPlugin object and loop through checking for        */
/*           data on the websockets and on the ROS topics.   spin_some is used     */
/*           implement non-blocking ROS communications.                            */
/***********************************************************************************/  

#include <sstream>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include "cellular_driver/CellularDriver.hpp"

int main(int argc, char** argv)
{


  std::cout << "starting up\n";
  rclcpp::init(argc, argv);
  auto sp = std::make_shared<cellular_driver::CellularDriver>();
  sp->initialize();
  while (rclcpp::ok())
  {
    sp -> processWebsockets();
    rclcpp::spin_some(sp);
    sleep(1);
  }
  rclcpp::shutdown();

  return 0;
}
