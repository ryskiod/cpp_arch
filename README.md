# cpp_arch

# Goal

ROS2上で画像処理を行うC++ノードの実装を通して、C++のソフトウェア設計を体系的に学習するためのリポジトリ。 
アルゴリズム実装だけでなく、**依存分離・抽象化・柔軟な設計構造の構築**にフォーカス。

C++による画像処理アプリケーションを通して、以下のような設計力を身につける：

- ソフトウェア設計原則（OOP, SRP, OCPなど）の実践
- インターフェースによる依存抽象化
- 拡張性を意識した責務分離構成
- C++固有のスマートポインタ・所有権管理
- 各種デザインパターンの習得と適用

---

## 使用技術

- `ROS2 (Jazzy)`
- `C++17`
- `OpenCV`
- `cv_bridge`
- `ament_cmake`

---



## image_processor_package

## 実装済みの設計パターン・原則
- **Strategy パターン**
  - `IProcessor`により処理の戦略を切替可能`
  - `IProcessor`← `PassthroughProcessor`
- **Factoryパターン**
  - `文字列から処理クラスを生成`
  - `ProcessorFactory`
- **Dependency Injection**
  - `main.cpp`からノードに処理器を注入`
- **OCP（開放/閉鎖原則）**
  - `新しいProcessor追加時、既存コードを変更不要`
- **ユニーク所有権管理**
  - ``unique_ptr<IProcessor>`で責任を明確化`

## image_source_bridge_package

## 実装済みの設計パターン・原則
- **Bridge パターン**
  - `IImageSource` ← `CameraSource`, `MockSource`
- **Factory パターン**
  - `ImageSourceFactory`, `ProcessorFactory`