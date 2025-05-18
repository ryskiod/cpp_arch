# image_processor_package

## 構成概要

- `IProcessor`：画像処理の抽象インターフェース（純粋仮想）
- `PassthroughProcessor`：何もしない処理（基本の実装例）
- `ImgSubscriber`：ROS2ノードとしてImageトピックを購読し、処理器を通して表示
- `ProcessorFactory`：文字列指定でProcessorの具体型を生成
- `main.cpp`：FactoryからProcessorを生成し、ノードに注入して実行

---

## 実装済みの設計パターン・原則

- **Strategy パターン**
  - `IProcessor`により処理の戦略を切替可能`
- **Factoryパターン**
  - `文字列から処理クラスを生成`
- **Dependency Injection**
  - ``main.cpp`からノードに処理器を注入`
- **OCP（開放/閉鎖原則）**
  - `新しいProcessor追加時、既存コードを変更不要`
- **ユニーク所有権管理**
  - ``unique_ptr<IProcessor>`で責任を明確化`

