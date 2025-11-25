# ComposableNodeを使った実装例

## Step1 カスタムメッセージを作る

1. msgを作成
2. CMakeLists.txtを修正
3. package.xmlを修正
4. 確認

### 1. msgを作成

今回はカスタムmsg用にパッケージを用意する。`address_value_msg`パッケージを作っていく。

```shell
ros2 pkg create address_value_msg
```

address_value_msgパッケージに移動し 以下の内容で`msg/AddressValue.msg`を作成する。

```AddressValue.msg
int32 address
int32 value
```

### 2. CMakeLists.txtを修正

カスタムmsgを作るために以下を追記する

```CMakeLists.txt 追記
# ++++ for custom msgs ++++ 
find_package(rosidl_default_generators REQUIRED)
set(msg_files
  "msg/AddressValue.msg"
)
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)
# ++++ for custom msgs ++++ 
```

全体はこのような感じ。

```CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(address_value_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# ++++ for custom msgs ++++ 
find_package(rosidl_default_generators REQUIRED)
set(msg_files
  "msg/AddressValue.msg"
)
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)
# ++++ for custom msgs ++++ 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

### 3. package.xmlを修正

`package.xml`に以下を追記

```package.xml 追記
  <!-- ++++ for custom msgs ++++ -->
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  <!-- ++++ for custom msgs ++++ -->
```

全体はこのような感じ。

```package.xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>address_value_msg</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="chocolate0785lego@icloud.com">tomixrm</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <!-- ++++ for custom msgs ++++ -->
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  <!-- ++++ for custom msgs ++++ -->
   
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 4. 確認

1. `colcon build`でビルドして実際にmsgが作られるかチェック
2. `ros2 interface show`でカスタムメッセージが登録されているかチェック

#### ビルドできるかチェック

```shell
colcon build --packages-select address_value_msg
```

無事成功すれば次に進む

#### `ros2 interface show`でチェック

`ros2 interface show <パッケージ名/msg/メッセージファイル名>`で検証

```shell
ros2 interface show address_value_msg/msg/AddressValue
```

無事認識されると以下のような表示になる

```shell
❯ ros2 interface show address_value_msg/msg/AddressValue
int32 address
int32 value
```

