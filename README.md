# LD06_test
nkym_hrkさんへ

ArduinoはLD06 LiDARからUART（230400 bps）で送信されるデータを受信し、USBシリアル通信を介してPCへそのまま転送する。Pythonは受信した47バイトのデータパケットを解析し、回転速度、開始角度、終了角度、12点分の距離（mm）および信頼度を取得する。各測定点の角度は開始角度と終了角度から線形補間して求め、距離と角度を用いて極座標から直交座標（x, y）へ変換する。その後、OpenCVでLiDARを中心とした360°の点群としてリアルタイムに描画し、障害物や壁の位置を可視化する。プログラムは終了操作（Qキー、Escキー、ウィンドウを閉じる、またはCtrl+C）が行われるまで、データの受信・解析・座標変換・描画を繰り返し実行する。測定点には信頼度が含まれるため、信頼度の低いデータを除外することで、より安定した表示が可能である。

ということで角度もわかるらしい。知らなかった(๑>؂<๑)
レーダーみたくしてみました。

<img width="783" height="825" alt="image" src="https://github.com/user-attachments/assets/80adc21a-9806-491c-8c3c-41b5690b3a24" />

COMポートなどは、デバイスマネージャー等で見つけて下さい
また、ArduinoIDEなどシリアルポートを他で開いてみるとエラーが出て落ちます。

:: プロジェクトフォルダ作成
mkdir LD06
cd LD06

:: 仮想環境作成
python -m venv venv

:: 仮想環境有効化
venv\Scripts\activate

:: pip最新版へ更新（推奨）
python -m pip install --upgrade pip

:: 必要ライブラリをインストール
pip install pyserial numpy opencv-python

:: インストール確認
pip list

:: Pythonのバージョン確認
python --version

:: プログラム実行
python LD06_test.py


:: 仮想環境終了
deactivate
