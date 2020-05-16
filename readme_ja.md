@page readme_ja.md
# Arduino-I2C-KM1
Arduino/Genuino ボードから KeiganMotor を I2C 通信でコントロールするためのライブラリです。
ESP32 または ESP8266 (Espressif 社) からも使用可能です.

## Github
- https://github.com/keigan-motor/Arduino-I2C-KM1

## ドキュメント
- https://docs.keigan-motor.com/apiDocument/Arduino-I2C-KM1/readme_ja_8md.html
- https://docs.keigan-motor.com

## 必要条件
- Arduino UNO / Mega / Mega 2560, ESP8266 / ESP32 シリーズ, M5Stack
- KeiganMotor KM-1 シリーズ
 - ファームウェアバージョンは、2.36 以上必須

## インストール

    $ git clone https://github.com/keigan-motor/Arduino-I2C-KM1

ライブラリマネージャからもインストール可能です（githubよりも古い場合があります）。 キーワード： "keigan".

### Zip

    https://github.com/keigan-motor/Arduino-I2C-KM1/archive/master.zip
    
## 接続方法
KeiganMotor の I2C ポートの場所については、下記ページを参照下さい。
- https://document.keigan-motor.com/software_dev/ports_on_wire

KeiganMotor のI2Cポートからヘッダーピンを使用するハーネス（オプション）
- https://keiganmotor.myshopify.com/products/i2c


***NOTE***
プルアップ抵抗について、以下を参照下さい。


### プルアップ抵抗
1kΩから10kΩ程度の、外部プルアップ抵抗を使用して下さい。使用する箇所は、以下となります。(Arduino のソフトウェア INPUT_PULLUP は推奨しませんが、動作する場合もあります。)
- SDA（データライン）と 3.3V の間 // Default SDA = 21
- SCL（クロックライン）と 3.3V の間 // Default SCL = 22

***NOTE***
KeiganMotor は、通信ラインに 3.0V を使用しています。
Arduino UNO のような 5V 動作のマイコンから駆動する場合は、KeiganMotor のマイコンを保護し、
通信エラーを低減するため、ロジックレベル変換を行って下さい。以下のようなものをお勧めします。
- https://www.switch-science.com/catalog/1523/

<img src="https://github.com/keigan-motor/Arduino-I2C-KM1/blob/master/img/levelshifter.jpg?raw=true" width="400">

|Level shifter |Arduino UNO|KeiganMotor_I2C|
|---|---|---|
|HV|5V     |-  |
|HV1 |SDA(21)|-  |
|HV2 |SCL(22)|-  |
|LV|3.3V   |-  |
|LV1 |-      |SDA|
|LV2 |-      |SCL|
|GND  |GND    |GND| 

PCA9306 を使用したレベル変換モジュール http://akizukidenshi.com/catalog/g/gM-05452/ を使用する場合, 裏面の J4, J5 ジャンパーをカッターナイフなどでカットして下さい。本モジュールを動作させるため、基板上のプルアップ抵抗 1kΩでは、KeiganMotor 側のドライブ能力が不足します。外部プルアップ抵抗がなくても正常動作する場合がありますが、必要に応じて、4kΩ以上の外部プルアップ抵抗を追加して下さい。

<img src="https://github.com/keigan-motor/Arduino-I2C-KM1/blob/master/img/pca9306_top.jpg?raw=true" width="400">
<img src="https://github.com/keigan-motor/Arduino-I2C-KM1/blob/master/img/pca9306_bottom.jpg?raw=true" width="400">

接続は以下となります。VREF1側は必ず電圧3.3Vの KeiganMotor として下さい。（PCA9306仕様による）
|PCA9306 |Arduino UNO|KeiganMotor_I2C|
|---|---|---|
|VREF1|3.3V     |-|
|SDA1 |-|SDA|
|SCL1 |-|SCL|
|VREF2|5V   |-  |
|SDA2|SDA(21)|-|
|SCL2|SCL(22)|SCL|
|GND  |GND    |GND| 

#### M5Stack
KeiganMotor を直接接続可能です。
外部プルアップ 3.3kΩ が予め SDA, SCL ラインに実装されており、ロジックレベルが 3.3Vであるためです。（通常の ESP32, ESP8266 では、外部プルアップ抵抗が必要です）

<img src="https://github.com/keigan-motor/Arduino-I2C-KM1/blob/ver2/img/M5Stack_connection.jpg?raw=true" width="600">

|M5Stack |KeiganMotor_I2C|
|---|---|
|SDA(21)|SDA|
|SCL(22)|SCL|
|GND|GND|

***NOTE***
KeiganMotor I2C ポートの 5V ピンは接続不要です。


## 基本
ライブラリをインクルードして、KeiganMotor インスタンスを初期化してください。

### ライブラリのインクルード
```arduino
#include "KM1_I2C.h"
```
### I2Cスレーブアドレスで初期化
#### KeiganMotor KM-1 シリーズの デフォルト I2C アドレスは、"0x20" 
これにより、I2C通信が自動的に初期化、スタートされます。
```arduino
KeiganMotor motor(0x20); // It includes wire.begin();
```
***NOTE***
I2Cアドレス "0xA0" は、0x20 と同様の結果となります。I2C アドレスは 7bitの制限のためです.

### 回転
```arduino
motor.enable();
motor.speedRpm(10);
motor.runForward();
delay(5000);
motor.runReverse();
delay(5000);
motor.stop();
```

### I2Cアドレスの変更
```arduino
motor.i2cSlaveAddress(0xB0);
motor.saveAllRegisters();
delay(2000);
motor.reboot();
```

## Examples
### "examples" フォルダをご覧ください。

|Example  |説明  |
|---|---|
|Scan.ino  |全てのI2Cデバイスをスキャンし、KeiganMotorを特定します|
|ChangeI2CAddress.ino  |I2Cスレーブアドレスを変更します|
|Run.ino  |回転させます（速度制御）|
|MoveTo.ino  |絶対位置へ移動します (位置制御)|
|MoveBy.ino  |一定距離移動します (相対位置へ移動) (位置制御)|
|Torque.ino  |最大トルクと位置の制御を行います|
|ReadMotorMeasurement.ino  |モーター測定値を読み取ります (位置、速度、トルク)|
|Dual.ino  |２つの KeignaMotor を同時に回転させます|
|Reset.ino  |全てのレジスタ（設定値）をリセットし、保存します|
|Wave.ino  |cosine波（余弦波）の位置制御をタイマーを使って行います|
|PID.ino  |PIDパラメータを読み取ります|

## 履歴
- ver 2.0.2 ESP32 または M5Stack について、コンパイルエラーのバグ修正
- ver.2.0.1 I2C接続について修正
- ver.2.0.0 メジャーアップデート

## 作成者

[@tkeigan](https://twitter.com/tkeigan)
Keigan Inc.

## ライセンス

[MIT](http://b4b4r07.mit-license.org)
