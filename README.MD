# ros2-component-template

- `src/component_template/` から対話的に新しい ROS 2 コンポーネントパッケージを生成できるテンプレートです。
- Publisher/Subscriber/Timer のサンプル実装（`std_msgs::msg::Int32`）と、適切に設定された `CMakeLists.txt` / `package.xml` を含みます。
- `rclcpp_components` によるコンポーネント登録まで済んでいるので、生成後すぐに `colcon build` して動かせます。

## クイックスタート：コンポーネント生成

1. 依存: ROS 2 環境と `colcon` が使えること。
2. テンプレートディレクトリへ移動しスクリプト実行:

   ```bash
   cd src/component_template
   ./create_component.sh
   ```

3. 対話で以下を入力（空欄はデフォルト適用）:
   - Package name（snake_case）
   - Class name（PascalCase）
   - Description（package.xml 用）
   - Node name / Executable name / Namespace
4. 設定サマリが表示されたら `y` で生成。
5. ルートに戻ってビルド＆セットアップ:

   ```bash
   colcon build --packages-select <package_name>
   source install/setup.bash
   ```

6. 実行:

   ```bash
   ros2 run <package_name> <executable_name>
   ```

## 生成される構成（例）

```tree
src/
├── component_template/
│   └── ... (テンプレート本体)
└── <package_name>/
    ├── CMakeLists.txt
    ├── package.xml
    ├── include/<package_name>/<package_name>.hpp
    └── src/<package_name>.cpp
```
