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

## ComposableNodeのLaunchサンプル

このテンプレートには、複数の Composable Node を単一のコンテナで起動するサンプル Launch ファイルが含まれています（`src/sample_bringup/launch/sample.launch.py`）。

### サンプルの実行方法

```bash
colcon build
source install/setup.bash
ros2 launch sample_bringup sample.launch.py
```

### Launch ファイルの構成

サンプルでは、2つの `component_template` ノードを `component_container_mt`（マルチスレッドコンテナ）で起動します:

- **component_template_node_A**
  - namespace: `sample`
  - topic: `example_int_A`
  - 初期カウント: 0
  - 発行間隔: 100ms

- **component_template_node_B**
  - namespace: `sample`
  - topic: `example_int_B`
  - 初期カウント: 1000
  - 発行間隔: 1000ms

### 主な設定ポイント

- **`ComposableNode`**: 各ノードの設定（package、plugin、name、namespace、parameters）
- **`ComposableNodeContainer`**: 複数ノードをまとめて管理するコンテナ
  - `executable='component_container_mt'`: マルチスレッド実行（シングルスレッド版は `component_container`）
- **`use_intra_process_comms: True`**: プロセス内通信を有効化（低レイテンシ・ゼロコピー）

### 自分のコンポーネントで使う場合

生成したコンポーネントを Launch で起動するには:

1. `package` を生成したパッケージ名に変更
2. `plugin` を `<namespace>::<ClassName>` 形式で指定
3. `parameters` に宣言したパラメータを設定
4. 必要に応じてコンテナの `executable` をシングル/マルチスレッド切り替え

```python
my_node = ComposableNode(
    package='my_package',
    plugin='my_package::MyClass',
    name='my_node',
    namespace='my_namespace',
    parameters=[{'param_name': value}],
    extra_arguments=[{"use_intra_process_comms": True}],
)
```

## Makefile コマンド一覧

ルートディレクトリの `Makefile` で、よく使うコマンドを簡潔に実行できます:

| コマンド            | 説明                                                      |
| ------------------- | --------------------------------------------------------- |
| `make help`         | 利用可能なコマンド一覧を表示                              |
| `make build`        | 全パッケージをビルド（`--symlink-install` 付き）          |
| `make format`       | C++ ファイルを clang-format で自動整形                    |
| `make check-format` | フォーマットチェック（ファイル変更なし）                  |
| `make clean`        | ビルド成果物を削除（`build/`, `install/`, `log/`）       |
| `make test`         | 全テストを実行し、結果を詳細表示                          |
