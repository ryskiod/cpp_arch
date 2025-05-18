# Type Erased Processor

画像処理における柔軟な処理戦略の切り替えを可能にする、**Type ErasureベースのROS2ノード実装**。
`std::function` を活用し、仮想関数やクラス継承に依存せず、任意の呼び出し可能な処理を動的に注入・実行できる構成を採用。

---

## 目的

* 継承階層に依存しない柔軟な戦略切り替え（Strategyの脱構造化）
* 型に依存しない処理抽象の導入（Type Erasure）
* `main()`で制御を完結させる設計パターンの練習
* クラスベースのOOPと値ベース抽象との比較検討

---

## 構成概要

| ファイル                        | 役割                         |
| --------------------------- | -------------------------- |
| `main.cpp`                  | lamda定義、processor生成、node実行   |
| `img_subscriber.hpp / cpp`  | ROS2 node, 画像を購読し処理を実行        |
| `type_erased_processor.hpp` | `std::function` を保持し処理を抽象化 |
| `processor_factory.hpp`     | 名前に応じて処理内容（lamda）を返す         |

---

## Type Erasure

```cpp
TypeErasedProcessor processor([](const cv::Mat& img) {
    return img; // 任意の処理
});

cv::Mat result = processor.process(input);
```

この形式により、処理クラスを定義せず、ラムダや関数ポインタだけで戦略を構成可能。

---

## 実装済みの設計パターン・原則

* **Type Erasure**：`std::function` で動的戦略注入を実現 ✅
* **クラス継承の排除**：`IProcessor` を廃止。構造と処理を切り離し ✅
* **戦略の動的切替**：Factoryでの分岐・ラムダの切替 ✅
* **mainに集約された制御**：制御構造の可視性と拡張性を確保 ✅

---