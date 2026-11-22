# Streamer.botからLiveVR Studioを操作する

この手順書では、[Streamer.bot](https://streamer.bot/) から LiveVR Studio の外部制御（製品API）を使って、Twitch のチャンネルポイント引換やチャットコマンドをきっかけに、表情・背景の切り替えやダンスの再生などを実行する設定方法を説明します。

アプリ内のヘルプ（設定の節）にも概要があります。この手順書が全体の正本です。

## 仕組み

- Streamer.bot のアクションが、LiveVR Studio の制御API（`http://127.0.0.1:17661/command`）へ HTTP の POST リクエストを送ります。
- すべてのリクエストに API トークン（認証）が必要です。
- 送る内容は JSON で、`command` に命令の名前、続けて必要な引数を指定します。ホットキーに割り当てた操作は `hotkey.run` で実行できます。

## 前提

- LiveVR Studio と Streamer.bot を、**同じパソコン**で動かします。制御APIは同じパソコンの中からしか接続できません。
- 外部制御は**コラボDLC**に含まれます。有効にするにはコラボDLCの所有が必要です。
- Streamer.bot と Twitch の接続（アカウント連携）は済んでいるものとします。

## 手順1: LiveVR Studio 側の準備

1. LiveVR Studio を起動します。
2. 設定画面を開き、「外部制御」グループの「外部制御を有効にする」をオンにします。コラボDLCを所有していない場合は、このグループ自体が使えません（案内が表示されます）。
3. 状態表示が「待受中: http://127.0.0.1:17661/（トークン必須）」になったことを確認します。
4. API トークンの「コピー」ボタンを押して、トークンを控えます。トークンは通常は伏せて表示されます（「表示」ボタンで中身を確認できます）。

## 手順2: 実行できる操作を調べる

`hotkey.run` で実行できる項目は、LiveVR Studio の状態取得（`/status`）で確認できます。

1. トークンを付けて `http://127.0.0.1:17661/status` を開きます。curl では次のようにします（`<トークン>` は手順1でコピーしたものに置き換えます）。

   ```
   curl -H "Authorization: Bearer <トークン>" http://127.0.0.1:17661/status
   ```

2. `hotkeys.actions` に、実行できる項目が `kind`（種類）と `target`（対象）と `label`（名前）で一覧されます。`on` が付いている項目は、現在の状態（オン・オフ）も分かります。
3. 表情のキーは `expressions`、背景のキーは `backgrounds` に出ます。

## 手順3: Streamer.bot 側の設定

### 3-1. チャンネルポイント報酬を作る

1. Streamer.bot で **Platforms → Twitch → Channel Point Rewards** を開きます。
2. 一覧を右クリックして **Add** を選び、報酬の名前（例: 背景を変える）とポイント数を決めて保存します。

※ Twitch のダッシュボード側で作った報酬は Streamer.bot で完全には扱えない場合があるため、報酬は Streamer.bot 側で作ることをおすすめします。

### 3-2. アクションを作る

1. **Actions & Queues → Actions** を開き、右クリック → **Add** でアクションを作ります（名前の例: LiveVR 背景切替）。
2. 右上の **Triggers** で **Add → Twitch → Channel Reward → Reward Redemption** を選び、手順3-1で作った報酬を選びます。
3. 下段の **Sub-Actions** で右クリック → **Add → Core → C# → Execute C# Code** を選びます。
4. 次のコードを貼り付け、`token` の値を手順1でコピーしたトークンに置き換えます。`json` の中身が実行する操作です（この例は背景を次の背景へ切り替えます）。

   ```csharp
   using System;
   using System.Net.Http;
   using System.Text;

   public class CPHInline
   {
       private static readonly HttpClient httpClient = new HttpClient();

       public bool Execute()
       {
           // LiveVR Studio の API トークン（設定画面の「外部制御」でコピーしたもの）
           string token = "ここにトークンを貼る";
           // 実行したい操作（この例では背景を次の背景へ切り替え）
           string json = "{\"command\":\"hotkey.run\",\"kind\":\"background\",\"target\":\"@next\"}";

           using (var request = new HttpRequestMessage(HttpMethod.Post, "http://127.0.0.1:17661/command"))
           {
               request.Headers.TryAddWithoutValidation("Authorization", "Bearer " + token);
               request.Content = new StringContent(json, Encoding.UTF8, "application/json");
               using (var response = httpClient.SendAsync(request).GetAwaiter().GetResult())
               {
                   string body = response.Content.ReadAsStringAsync().GetAwaiter().GetResult();
                   CPH.LogInfo("LiveVR: " + (int)response.StatusCode + " " + body);
               }
           }
           return true;
       }
   }
   ```

5. アクションのテスト実行で LiveVR Studio の背景が切り替われば成功です。実際にチャンネルポイントを引き換えても確認できます。

※ Streamer.bot の「Fetch URL」サブアクションは GET 専用のため、POST が必要なこの手順では使いません。POST には上記のとおり Execute C# Code を使います（Streamer.bot 公式の案内も同じ方法です）。

## 設定例

### 背景を次の背景へ切り替える

```json
{"command":"hotkey.run","kind":"background","target":"@next"}
```

`target` には `@prev`（前の背景）、`@transparent`（透過）、`@video`（動画背景）も使えます。特定の背景に切り替えるときは、`target` に背景のキーを入れるか、次の命令を使います。

```json
{"command":"background.set","key":"<背景のキー>"}
```

### 表情を切り替える

```json
{"command":"expression.set","key":"<表情のキー>"}
```

表情のキーは `/status` の `expressions` で確認します。表情を戻すときは次の命令です。

```json
{"command":"expression.clear"}
```

### ダンスを再生する

```json
{"command":"hotkey.run","kind":"dance","target":"<ダンスの対象>"}
```

`target` は `/status` の `hotkeys.actions` で、`kind` が `dance` の項目を確認します。

### チャットコマンドでも使う

同じアクションに Triggers でチャットコマンドのトリガーを追加すると、チャンネルポイントとチャットコマンドの両方から同じ操作を実行できます。

## 使える命令の一覧

製品APIで使える命令は **207 件**です（バージョン 0.1.1 時点）。実行中の LiveVR Studio が受け付ける命令の一覧は、トークンを付けて `http://127.0.0.1:17661/` を開くと取得できます。実際に使える一覧はそちらが正本です（ビルドによってはコラボ関連など一部が含まれないことがあります）。各命令の引数は、代表的なものは「設定例」と `/status` の内容を参照してください。

### プラグイン命令（Stream Deck プラグインと共通）（24件）

`syphon.start`、`syphon.stop`、`panel.show`、`panel.close`、`spring.set`、`expression.set`、`expression.clear`、`emergency.hide`、`model.set_hidden`、`model.set`、`background.set`、`pose.set`、`pose.clear`、`calibration.face`、`calibration.body`、`calibration.combined`、`camera.set`、`microphone.set`、`away.set`、`virtualcamera.set`、`gamecapture.set`、`hotkey.run`、`custom_expression.key.set`、`custom_expression.key.get`

### 設定の読み取り（2件）

`settings.list`、`settings.get`

### 設定の書き込み（1件）

`settings.set`

### フレーム・画面の保存（2件）

`frame.save`、`screen.save`

### コラボ（38件）

`collab.connect`、`collab.host`、`collab.invite`、`collab.invite.cancel`、`collab.invite.accept`、`collab.invite.decline`、`collab.leave`、`collab.kick`、`collab.block`、`collab.unblock`、`collab.allow_rejoin`、`collab.placement.set`、`collab.placement.reset`、`collab.placement.reset_row`、`collab.formation.set`、`collab.placement_link.set`、`collab.voice.set`、`collab.voice.set_monitor`、`collab.voice.output.set`、`collab.voice.output.refresh`、`collab.dialog.answer`、`collab.transfer.cancel`、`collab.friends.refresh`、`collab.friends.tab`、`collab.friends.search`、`collab.friends.filter`、`collab.friends.others`、`collab.friends.window`、`collab.favorite.set`、`collab.history.invite`、`collab.bar.center`、`collab.bar.lock`、`collab.bar.lockall`、`collab.bar.display`、`collab.bar.formation`、`collab.bar.resetformation`、`collab.bar.positionlink`、`collab.bar.reorder`

### モデルの追加（3件）

`dance.register`、`model.add`、`model.cancel`

### モデルの削除（3件）

`dance.unregister`、`model.remove`、`model.remove_cancel`

### 背景の追加（2件）

`background.add`、`background.cancel`

### 背景の削除（2件）

`background.remove`、`background.remove_cancel`

### シーン（5件）

`scene.set`、`scene.save`、`scene.rename`、`scene.reset`、`scene.cancel`

### シーンの削除（3件）

`scene.delete`、`scene.delete_confirm`、`scene.delete_cancel`

### プロファイル（2件）

`profile.save`、`profile.restore`

### 姿勢制御・外部入力（41件）

`glove.set`、`leap.set`、`gaze.configure`、`gaze.submit`、`gaze.clear`、`ifacial.set`、`tracking.input.set`、`vmc.set`、`multicamera.configure`、`multicamera.refresh`、`multicamera.set`、`multicamera.run`、`multicamera.set_mode`、`multicamera.slot_settings.open`、`multicamera.slot_settings.close`、`multicamera.clock`、`multicamera.calibration.start`、`multicamera.calibration.stop`、`multicamera.calibration.solve`、`multicamera.calibration.board`、`multicamera.wizard.open`、`multicamera.wizard.close`、`multicamera.wizard.step`、`multicamera.wizard.set_count`、`multicamera.wizard.set_type`、`multicamera.wizard.select_slot`、`multicamera.wizard.set_placement`、`multicamera.wizard.set_device`、`multicamera.wizard.set_choice`、`multicamera.wizard.finish`、`multicamera.wizard.calibration_start`、`multicamera.wizard.calibration_stop`、`multicamera.wizard.calibration_solve`、`tdpt.set`、`imu.set`、`tracking.pose.face.set`、`tracking.pose.face.get`、`tracking.pose.face.clear`、`tracking.pose.body.set`、`tracking.pose.body.get`、`tracking.pose.body.clear`

### その他の操作（79件）

`dance.play`、`dance.stop`、`dance.select`、`dance.set_loop`、`dance.set_music`、`dance.set_timing`、`dance.set_camera`、`dance.set_camera_enabled`、`dance.settings.open`、`dance.settings.close`、`updater.check`、`updater.open_target`、`panel.section.set`、`background.set_video_limit`、`framing.set`、`framing.reset`、`framing.center_face`、`framing.lock.set`、`framing.stage.set`、`framing.focus.set`、`framing.focus.scale`、`multiangle.set_open`、`multiangle.register`、`multiangle.select`、`multiangle.regenerate`、`multiangle.rename`、`tutorial.start`、`background.set_playing`、`colorblend.set`、`lighting.set_enabled`、`lighting.set_type`、`lighting.add`、`lighting.remove`、`lighting.reset`、`attachments.remove`、`attachments.set_pose`、`model.set_favorite`、`model.set_favorites_only`、`model.move`、`help.show`、`help.show_section`、`help.show_terms_article`、`help.search`、`hotkeys.set_enabled`、`hotkeys.set_global_enabled`、`hotkey.assign`、`hotkey.set_enabled`、`hotkey.add`、`hotkey.remove`、`hotkey.add_cancel`、`hotkey.run_row`、`custom_expression.save`、`custom_expression.apply`、`custom_expression.clear`、`parameter_mapping.set`、`parameter_mapping.remove`、`calibration.hand`、`microphone.vowel_calibrate`、`microphone.vowel_calibrate_sequence`、`microphone.vowel_reset`、`microphone.vowel_cancel`、`mic.chain.reset`、`face.reset`、`attachments.add`、`attachments.select`、`tutorial.back`、`tutorial.skip`、`tutorial.abort`、`tutorial.confirm`、`startup.set_theme`、`startup.set_language`、`startup.answer_tutorial`、`startup.recovery_skip`、`startup.recovery_retry`、`startup.recovery_reselect`、`microphone.refresh_devices`、`minimap.center`、`stagegizmo.view`、`inputguide.status`


## 応答とエラー

| コード | 意味 |
| --- | --- |
| 200 | 実行できました。本文に結果の JSON が返ります |
| 400 | 要求の形式が正しくありません（JSON の書き間違い、必須の引数の不足など） |
| 401 | トークンが無いか、正しくありません |
| 403 | その命令は製品APIでは許可されていません（下記の「使えない命令」など） |
| 404 | その経路はありません（URL の書き間違いなど） |
| 504 | LiveVR Studio 側の処理が時間内に終わりませんでした |

## 使えない命令

製品APIでは、次のような命令は使えません（403 が返ります）。

- 検証・デバッグ用の命令（名前に `debug` や `test` を含むもの）
- アプリを終了する `app.quit`、プロファイルを切り替える `app.profile.switch`、初期化の復元 `app.factory_reset.restore`
- アップデートを実行する `updater.apply`（アップデートは画面から実行してください）

## 注意

- **トークンは合言葉です。** 配信画面やスクリーンショットに映らないよう扱ってください。Streamer.bot の設定にもトークンがそのまま入るため、設定ファイルの共有にも注意してください。漏れたときは設定画面の「トークンを再発行する」で再発行すると、古いトークンはすぐに使えなくなります。
- **接続先は 127.0.0.1（同じパソコンの中）だけです。** LAN やインターネットからは接続できません。ポートを外部へ公開する設定（ポート開放や転送）はしないでください。その必要もありません。
- トークンは設定の初期化でも消えません。再発行したときだけ変わります。
- 命令の引数の値は、すべて文字列で送ります。数値や `true` / `false` をそのまま書かず、`"1"` のように引用符で囲んでください。
