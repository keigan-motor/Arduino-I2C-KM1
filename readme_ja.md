@page readme_ja.md
# Arduino-I2C-KM1
Arduino/Genuino ボードから KeiganMotor を I2C 通信でコントロールするためのライブラリです。
ESP32 または ESP8266 (Espressif 社) からも使用可能です.

- 英語版の説明はこちら → @ref index

## Github
- https://github.com/keigan-motor/Arduino-I2C-KM1

## ドキュメント
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
- http://akizukidenshi.com/catalog/g/gM-05452/
    - 本モジュール（PCA9306）と Arduino を用いた接続は、以下のようになります。
    
|PCA9306|Arduino UNO|KeiganMotor_I2C|
|---|---|---|
|VREF1|5V     |-  |
|SDA1 |SDA(21)|-  |
|SCL1 |SCL(22)|-  |
|VREF2|3.3V   |-  |
|SDA2 |-      |SDA|
|SCL2 |-      |SCL|
|GND  |GND    |GND|


#### M5Stack
KeiganMotor を直接接続可能です。
外部プルアップ抵抗が予め実装されており、ロジックレベルが 3.3Vであるためです。
（通常の ESP32, ESP8266 では、外部プルアップ抵抗が必要です）

- 接続は以下のようになります。
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



## 作成者

[@tkeigan](https://twitter.com/tkeigan)
Keigan Inc.

## ライセンス

[MIT](http://b4b4r07.mit-license.org)
