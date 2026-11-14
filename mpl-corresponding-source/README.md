# MPL-2.0 対応ソースの提供

このフォルダは、LiveVR Studio に含まれる Mozilla Public License 2.0（MPL-2.0）の対象ファイルについて、ライセンスが求める対応ソース（Corresponding Source）を提供するためのものです。

## 対象ファイル

| ファイル | 内容 |
|---|---|
| `MediaPipeUpperBodyInput.cs` | ウェブカメラによる上半身トラッキングの入力実装。SysMocap（avatarRenderer.js）から adapted した要素を含みます |
| `WmcUpperBodyMath.cs` | 上半身パイプライン共用の計算集。60Hz 換算の追従計算式を含みます |

上記2ファイルは MPL-2.0 のもとで提供します。ライセンス文は同フォルダの `LICENSE-MPL-2.0.txt` です。

## 範囲について

LiveVR Studio 本体のソースコードは公開していません。このフォルダで提供するのは、上記の対象ファイルとライセンス文、本文書だけです。

## 経緯

同梱物のライセンス棚卸しで SysMocap 由来の要素を同定し、MPL-2.0 の対象として扱うことを決定しました（2026-11-15）。対応の記録は製品側の課題管理にあります。
