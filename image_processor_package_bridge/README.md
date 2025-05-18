# image_source_bridge_package

Bridgeパターンに基づいたC++ / ROS2 による画像処理サンプル。
抽象インターフェースとFactoryパターンを組み合わせて、拡張性・テスト性に優れた構成を目指す。

---

## Goal

- ✅ 複数の画像ソース（実カメラ / モック）を切り替えて使用可能にする
- ✅ 処理戦略（Strategy）を柔軟に変更可能にする
- ✅ main() は実装に依存せず、抽象型だけを扱う
- ✅ テストのしやすい構成を作る（MockSource導入）

---

## Architecture Overview

- **Bridge パターン**
  - `IImageSource` ← `CameraSource`, `MockSource`
- **Factory パターン**
  - `ImageSourceFactory`, `ProcessorFactory`

main関数がこれらの抽象を注入して、責務を明確に分離する。

```txt
+---------------------------+
|        main.cpp          |
+---------------------------+
         |         |
         v         v
+--------------------------+     +--------------------------+
|      IImageSource        |     |        IProcessor        |
+--------------------------+     +--------------------------+
         ^                             ^
         |                             |
+--------------------+         +--------------------------+
|   CameraSource      |         |  PassthroughProcessor    |
|   MockSource        |         |  CannyProcessor (future) |
+--------------------+         +--------------------------+
```

---

## Bridgeパターンとは
抽象（Abstraction）と実装（Implementation）を分離し、それぞれを独立に拡張可能にする設計パターン。

抽象： 何をするか、どのように流れを制御するかを定義
実装： 実際にどう処理するかだけに責任を持つ
目的： 実装の詳細を隠蔽し、使い方（制御）と切り離すことで、柔軟かつ保守性の高い設計を実現する

### 本質的な特徴
Bridgeパターンの本質は、単なる構造の分離にとどまらず、以下の点にある：

抽象側（Abstraction）が処理の制御を担う
実装側（Implementation）は“どう動くか”のみに集中
実装はどこで使われるかを意識せず、抽象が“いつ／何をするか”を定義する

### このリポジトリにおけるBridgeの適用例
cpp
auto source = create("mock");
while (rclcpp::ok()) {
    if (auto frame = source->get_frame()) {
        ...
    }
}
- IImageSource が Bridgeの抽象クラス
- CameraSource, MockSource が 実装クラス
- main.cpp が 制御を担うAbstractionの役割
実装は get_frame() をどう返すかだけ知っており、制御の流れには関与しない

### 補足
実装と抽象の分離に加え、「制御の責任を抽象側に持たせること」で、Bridgeは真に効果を発揮する。
こうすることで実装は単純・自己完結になり、拡張・差し替えが容易になる。

## 🔧 Build

```bash
colcon build --packages-select image_source_bridge_package
source install/setup.bash

---

## 今後の拡張予定（設計力強化）

- [✅] 実装/抽象の独立化（Bridgeパターン）
- [ ] Type Erasureを使った汎用処理器（型抹消構成）
- [ ] External Polymorphismの導入
