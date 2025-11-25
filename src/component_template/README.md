# Component Template

ROS2コンポーネントパッケージのテンプレートです。このテンプレートから新しいコンポーネントパッケージを簡単に生成できます。

## テンプレートの内容

このテンプレートには以下が含まれています：

- 基本的なROS2コンポーネントクラス
- Publisher/Subscriber/Timerのサンプル実装
- `std_msgs::msg::Int32`を使ったメッセージのPublish/Subscribe
- 適切なCMakeLists.txtとpackage.xml設定
- rclcpp_componentsを使ったコンポーネント登録

## 新しいコンポーネントパッケージの作成方法

### 1. スクリプトを実行

```bash
cd src/component_template
./create_component.sh
```

### 2. 対話形式で情報を入力

スクリプトが以下の項目を順番に聞いてきます：

1. **Package name** (パッケージ名)
   - snake_caseで入力してください（例: `my_robot_controller`）
   - 小文字、数字、アンダースコアのみ使用可能

2. **Class name** (クラス名)
   - PascalCaseで入力してください（例: `MyRobotController`）
   - 大文字で始まるキャメルケース

3. **Description** (パッケージの説明)
   - package.xmlに記載される説明文
   - 空欄の場合は "ROS2 component package" がデフォルト

4. **Node name** (ノード名)
   - ランタイム時のノード名
   - 空欄の場合はパッケージ名と同じになります

5. **Executable name** (実行可能ファイル名)
   - `ros2 run`で使用する実行ファイル名
   - 空欄の場合は `${package_name}_node` になります

6. **Namespace** (名前空間)
   - C++の名前空間
   - 空欄の場合はパッケージ名と同じになります

### 3. 確認して生成

設定内容が表示されるので、確認して `y` を入力すると生成されます。

### 4. ビルドと実行

```bash
# ワークスペースのルートに移動
cd /home/tomixrm/ros2_workspace/learn_ws

# ビルド
colcon build --packages-select <your_package_name>

# セットアップ
source install/setup.bash

# 実行
ros2 run <your_package_name> <your_executable_name>
```

## 生成されるファイル構造

```tree
src/
├── component_template/
│   ├── create_component.sh
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/
│   └── src/
└── your_new_package/                # 生成されたパッケージ
    ├── CMakeLists.txt               # プロジェクト設定が置換済み
    ├── package.xml                  # パッケージ情報が置換済み
    ├── include/
    │   └── your_new_package/
    │       └── your_new_package.hpp # ヘッダーファイル
    └── src/
        └── your_new_package.cpp     # 実装ファイル
```

## 自動的に置換される項目

スクリプトは以下の項目を自動的に置換します：

- パッケージ名（`component_template` → あなたの指定した名前）
- クラス名（`ComponentTemplate` → あなたの指定した名前）
- Namespace名（`component_template` → あなたの指定した名前）
- ノード名（コンストラクタ内）
- Description（package.xml内）
- 実行可能ファイル名（CMakeLists.txt内）
- インクルードガード（ヘッダーファイル）
- ファイル名とディレクトリ名

## 使用例

```bash
$ cd src/component_template
$ ./create_component.sh

=== ROS2 Component Template Generator ===

Please enter the following information:

Package name (snake_case, e.g., my_robot_controller): my_sensor_driver
Class name (PascalCase, e.g., MyRobotController): MySensorDriver
Description (e.g., Robot controller for navigation): Driver for custom sensor
Node name (default: my_sensor_driver):
Executable name (default: my_sensor_driver_node):
Namespace (default: my_sensor_driver):

=== Configuration Summary ===
Package name:    my_sensor_driver
Class name:      MySensorDriver
Description:     Driver for custom sensor
Node name:       my_sensor_driver
Executable name: my_sensor_driver_node
Namespace:       my_sensor_driver

Continue with these settings? (y/n): y

=== Package created successfully! ===
```

## カスタマイズ

生成されたパッケージはテンプレートのコピーなので、自由に編集してください：

- メッセージ型の変更（`std_msgs::msg::Int32` → 他のメッセージ型）
- Publisher/Subscriberのトピック名変更
- タイマー周期の変更
- 依存パッケージの追加（package.xmlとCMakeLists.txt）
- 新しい機能の追加

## 注意事項

- 生成先のディレクトリが既に存在する場合、上書き確認が表示されます
- スクリプト実行後は、必ず `colcon build` でビルドしてください
- テンプレート自体（`component_template/`）は変更されません
