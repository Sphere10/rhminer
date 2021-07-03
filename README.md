# rhminer 

RandomHash miner for the PascalCoin blockchain.<br>
Support Intel/AMD 64 bit CPU and NVidia GPU.<br>
Support stratum and solo mining<br>
Works on Windows 7,10 and Ubuntu 18

## Download prebuilt binaries
**Current version is RHMINER_VERSION** <br>

There is one prebuilt binariy per OS and CUDA architectures. <br>
https://github.com/polyminer1/rhminer/releases/<br>

## Alternative download site : 
* Windows binaries https://mega.nz/#F!DqpAjCJQ!Q12a_YRlu_CWA92kIglKug
* Linux binaries https://mega.nz/#F!Dz4ElAwK!gbWbU4OpmEf6YnOCLIKfSQ


## Mining locally/Solo
To mine locally/solo you'll need the official PascalCoin wallet https://github.com/PascalCoin/PascalCoin/releases <br>
In order to mine locally with rhminer, **You need to set a miner name smaller than 26 characters and mine into a private key with encryption type secp256k1**<br>
The best way to assure you're mining in a secp256k1 private key is to create one and select it in the option **"Always mine with this key"**.<br>
**Do not use the "Use random existing key" option** because if one of your key is not a secp256k1 key, the miner will exit when. Plus when there is to much keys in the wallet it gives out errors, sometimes, when submiting nonces<br>
To ensure your miner name is correct, go to Project menu, then Options and set a miner name smaller than 26 characters<br>
To get the number of logical cores, on you system, simply run rhminer with the -completelist option. The last line is the cpu description with the amount of logical core. Ex :<br>
```
C:\rhminer>rhminer -completelist

  rhminer vRHMINER_VERSION beta for CPU by polyminer1 (https://github.com/polyminer1/rhminer)
  Buid CPU Nov 19 2018 20:04:01

  Donations : Pascal account 529692-23
  Donations : Bitcoin address 19GfXGpRJfwcHPx2Nf8wHgMps8Eat1o4Jp

CPU : Intel(R) Core(TM) i5-4460 CPU @ 3.20GHz with 4 logical cores
```
This tells you what is the ideal maximum number of threads for cpu mining (-cputhreads) <br>

```
Solo mining examples:  rhminer.exe -v 2 -r 20 -s http://127.0.0.1:4009 -cpu -cputhreads 1 -gpu 0 -gputhreads 100 -extrapayload HelloWorld

NOTE: remove -gpu 0 -gputhreads 100 if you dont have a gpu
```
Note2: It is not recommended to mine using a laptop.

## Supported Cuda architecture
* Kepler  GTX  700 series, Tesla K40/K80
* Maxwell GTX  900 series, Quadro M series, GTX Titan X
* Pascal  GTX 1000 series, Titan Xp, Tesla P40, Tesla P4,GP100/Tesla P100 DGX-1
* Volta   GTX 1100 series (GV104), Tesla V100
* Turing  GTX 2000 series (GTX RTX, GTX 2060, GTX 2070, GTX 2080)

## Gpu mining
To mine using gpu you must provide the gpu numbers and the amount of threads for each gpu.<br>
If you only have one gpu, use **-gpu 0** and **-gputhreads {amount of threads}**<br>
If you have more than one gpus, you can see their number by executing the miner with the *list* option :<br> 
``` 
C:>rhminer -list 

  rhminer v0.9 beta for CPU and NVIDIA GPUs by polyminer1 (http://github.com/polyminer1)
  NVIDIA CUDA SDK 9.2

  Donations : Pascal account 529692-23
  Donations : Bitcoin address 19GfXGpRJfwcHPx2Nf8wHgMps8Eat1o4Jp

List of gpus and cpus:
GPU0 : GeForce GTX 1060 3GB
GPU1 : GeForce GTX 1060 3GB
GPU2 : GeForce GTX 950 2GB
CPU  : Intel(R) Core(TM) i5-4460 CPU @ 3.20GHz
```
Then select the ones you want to mine with like this : <br> 
```
rhminer -s http://localhost:4009 -gpu 1,2 -gputhreads 262,102
```

## Ideal CUDA threads count
To find the ideal maximum amount of threads, start with 75% of the memory divided by 8.8. <br>
For a GTX 1060 3GB that is 3000 * 0.75 / 8.8 = 255 threads. <br>
Then run 2 minutes and if everything is stable, raise by say 32 until you get no crashes after 2 min.<br>
To help you in that process, look for the log line that say "CUDA: Using " when the miner starts. It  will indicate how much memory you take and how much is left depending on your selected thread count.<br>
**ALLWAYS** let at lease 150 meg of free memory, for internal OS operations, or you have stability issues.<br>


## Tested on 
CPU: I3, I5, Core2, Xeon, Athlon <br>
GPU: GTX 950, GTX 1060, GTX 1070 <br>
CUDA: Linux CUDA 9.1, Windows CUDA 9.2 <br>

## Performances for V1.3

| Cpu/GPU                                |  OS        | Threads      | Speed in H/s |   Extra infos                         |
| ---------------------------------------|------------|-------------:|-------------:|---------------------------------------|
|  i5-7500 CPU @ 3.40GHz                 | Win10      |    4         |   1112       |                                       |
|  i5-3337U CPU @ 1.80GHz                | Win10      |    4         |    417       |                                       |
|  i5-2400 CPU @ 3.1 GHz                 | Win10      |    4         |    791       |                                       |
|  i7-4770                               | Linux      |    8         |   1070       |                                       |
|  i7-4770 @ 3.7 GHz                     | Win10      |    8         |   1344       | dual ddr3 1333 memory                 |
|  i7-4770K @ 4.5 GHz                    | Kali       |    8         |   1841       |                                       |
|  i7-7700K @ 4.7 GHz                    | Ubuntu     |    8         |   2107       | dual ddr4 3200 memory                 |
|  i7-4790K                              | Ubuntu     |    8         |   1760       | dual channel                          |
|  i7-4790K                              | Win10      |    8         |   1420       | dual channel                          |
|  i7-4980HQ                             | Ubuntu     |    8         |   1396       |                                       |
|  i7-4980HQ                             | Win10      |    8         |   1360       |                                       |
|  i7-5600U                              | Ubuntu     |    4         |   541        |                                       |
|  i7-5600U                              | Win7       |    4         |   492        |                                       |
|  i7-6700K @ 4.6 GHz                    | Ubuntu     |    8         |   1755       | dual ddr4 3200 memory                 |
|  Xeon(R) CPU W3520 @ 2.67GHz           | Win10      |    8         |    819       |                                       |
|  Xeon(R) CPU E5-2665 0 @ 2.40GH        | Win10      |   16         |   1645       |                                       |
|  Xeon(r) CPU E5-2630v4                 | ?          |   14         |   1900       |                                       |
|  Xeon(r) CPU E5-2420v2 on Hyper-V      | ?          |    8         |    800       |                                       |
|  Xeon(R) CPU X5650 @ 2.67GHz           | ?          |   24         |   2357       |                                       |
|  Xeon(R) CPU X5675                     | Win10      |   12         |   1419       | tripple channel                       |
|  Xeon(R) CPU X5675                     | Ubuntu     |   12         |   1734       | tripple channel                       |
|  Xeon 12 core ES v3 E5 @ 2.7 GHz       | Kali       |   25         |   3400       |                                       |
|  Xeon(R) Platinum 8168 CPU @ 2.70GHz   | ?          |   32         |   **7162**   |                                       |
|  Dual Xeon(R) X5675                    | Win10      |   24         |   2330       | tripple channel                       |
|  Dual Xeon(R) X5675                    | Ubuntu     |   24         |   2850       | tripple channel                       |
|  Ryzen 1800x                           | Win10      |   16         |   2560       |                                       |
|  Ryzen threadripper 1950X @ 3.4 Ghz    | Ubuntu     |   32         |   **5378**   | sseboost 1, Quad Channel ddr4 3200mhz |
|  Ryzen 2700 @ 3.4 GHz                  | Ubuntu     |    6         |   2856       | sseboost 1, Quad Channel ddr4 3200mhz |
|  Ryzen 2700x@ 4.09 GHz                 | Kali       |    8         |   3046       |                                       |
|  Core(TM) 2 QuadCore Q6600 @ 2.40GHz   | Win7       |    4         |    397       |                                       |
|  Intel Atom X7                         | Win10      |    4         |    240       |                                       |
|  Intel Pentium 4400 @3.3Ghz            | Win10      |    2         |    504       | dual ddr4 2400 memory                 |
|  Intel Pentium G3420                   | Ubuntu     |    2         |    445       |                                       |
|  Intel Celeron 3930 @2.9GHz            | Win10      |    2         |    447       | mono ddr4 2400 memory                 |
|  AMD PHENOM-II-X6 @ 3.25Ghz            | Win10      |    6         |    564       | oldgen, ddr3 memory                   |
|  AMD PHENOM-II-X4 @ 3.6 GHz            | Ubuntu     |    4         |    460       | oldgen, ddr3 memory                   |

**NOTE: I do not recommend to overclock your cpu. If you do it, it's at your own risk.**

## Performance for v.1.4 on CUDA

| GPU                                    |  OS        | Threads      | Speed in H/s |
| ---------------------------------------|------------|-------------:|-------------:|
|   gtx 950 2gb                          | Windows    |    140       |    140       |
|   Gtx 1060 3gb                         | Windows    |    280       |    305       |
|   Gtx 1070 8gb                         | Windows    |    384       |    467       |

note: raw is for raw performance on all hyper-threads. This does not represent real life performance.

## Build instructions (Windows)                      
Install VisualStudio 2017 with chose Platform Toolset v140 <br>
Install the lastest NVIDIA Display Driver <br>
Install the CUDA Toolkit 9.2 (or more) <br>
Install boost_1_64_0 and make libs using bjam (https://www.boost.org/doc/libs/1_64_0/more/getting_started/windows.html) <br>
Open solution, select target and compile <br>
Run <br>

## Build Linux (Ubuntu)
sudo apt-get install git build-essential cmake  <br>
install CUDA ToolKit <br>
Install and compile boost_1_64_0 <br>
Install jsoncpp <br>
git clone https://github.com/polyminer1/rhminer.git <br>
cd rhminer <br>
mkdir build <br>
cd build <br>
*To build for CUDA Pascal :* cmake -DRH_CPU_ONLY=OFF -DRH_DEBUG_TARGET=OFF -DRH_CUDA_ARCH=Pascal --target all ..  <br>
*To build for CPU only    :* cmake -DRH_CPU_ONLY=ON -DRH_DEBUG_TARGET=OFF --target all ..  <br>
make all <br>


## ScreenSaver
To download the screensaver go to release section here https://github.com/polyminer1/rhminer/releases and download **PascalCoinScreenSaver.zip** <br>
To install PascalCoin ScreenSaver simply right-click on file PascalCoinScreenSaver.scr and click "install" from the menu. <br>
Then you can configure it. <br>
For Laptop users it is *STROGLY* recommented to set only 1 thread in the scrensaver's config. <br>
 <br>
To set a mining password open regedit.exe and append your password command line, followed by a space, to the string located here : **Computer\HKEY_CURRENT_USER\Software\PascalCoin\ScreenSaver\extra** <br>
EX: -pw MyEmail@email.com <br>
Dont forget to put a space at the end. <br>


## Stability issues
Thre are some limitations on nvidia gpu to consider.

First, the kernel is not 100% stable in all settings. This mean you'll have to experiment to find the stable sweet spot in term of gputhreads. Maximum thread count does not mean maximum speed. Sometimes lower thread count will give you more stability and more speed also.

On multiple gpu rigs, it's NOT recommended to mine CPU at the same time. You'll have more kernel timeout error because the driver will lack cpu time.<br>
Also, it is recommented, on multiple GPU rigs, to run the miner in a loop in a batch file !

## Troubleshoot
On Windows 7/8/10, if you get the missing OpenCL.dll error you need to download it into rhminer's folder. (hint: You can safely get one with the Intel SDK on Intel's opencl website)


## Command line options
```
General options:
  -maxsubmiterrors      Stop the miner when a number of consecutive submit errors occured.
                        Default is 10 consecutive errors.
                        This is usefull when mining into local wallet.
  -extrapayload         An extra payload to be added when submiting solution to local wallet.
  -apiport              Tcp port of the remote api.
                        Default port is 7111.
                        Set to 0 to disable server.
                        Port is read-only by default. See API.txt for more informations
  -apipw                Api password for non read-only (miner_restart, miner_reboot, control_gpu, ..).
                        Default password is empty (read-only mode).
                        Note: must match ethman password
  -worktimeout          No new work timeout. Default is 60 seconds
  -displayspeedtimeout  Display mining speeds every x seconds.
                        Default is 10
  -logfilename          Set the name of the log's filename.
                        Note: the log file will be overwritten every time you start rhminer
  -configfile           Xml config file containing all config options.
                        All other command line options are ignored if config file given.
  -processpriority      On windows only. Set miner's process priority.
                        0=Background Process, 1=Low Priority, 2=Normal Priority, 3=High Priority.
                        Default is 3.
                        NOTE:Background Proces mode will make the console disapear from the desktop and taskbar. WARNING: Changing this value will affect GPU mining.
  -v                    Log verbosity. From 0 to 3.
                        0 no log, 1 normal log, 2 include warnings. 3 network and silent logs.
                        Default is 1
  -list                 List all gpu in the system
  -completelist         Exhaustive list of all devices in the system
  -diff                 Set local difficulyu. ex: -diff 999
  -processorsaffinity   On windows only. Force miner to only run on selected logical core processors.
                        ex: -processorsaffinity 0,3 will make the miner run only on logical core #0 and #3.
                        WARNING: Changing this value will affect GPU mining.
  -h                    Display Help
  -help                 Display Help
  -?                    Display Help

Optimizations options:
  -memoryboost          This option will enable some memory optimizations that could make the miner slower on some cpu.
                        Test it with -testperformance before using it.
                        1 to enable boost. 0 to disable boost.
                        Enabled, by default, on cpu with hyperthreading.
  -sseboost             This option will enable some sse4 optimizations.
                        It could make the miner slower on some cpu.
                        Test it with -testperformance before using it.
                        1 to enable SSe4.1 optimizations. 0 to disable.
                        Disabled by default. 
  -cputhrottling        Slow down mining by internally throttling the cpu. 
                        This is usefull to prevent virtual computer provider throttling vCpu when mining softwares are detected.
                        Min-Max are 0 and 99.

Gpu options:
  -cpu                  Enable the use of CPU to mine.
                        ex '-cpu -cputhreads 4' will enable mining on cpu while gpu mining.
  -cputhreads           Number of CPU miner threads when mining with CPU. ex: -cpu -cputhreads 4.
                        NOTE: adding + before thread count will disable the maximum thread count safety of one thread per core/hyperthread.
                        Use this option at your own risk.
  -gpu                  Enable indiviaual GPU by their index. GPU not in the list will be disabled. ex: -gpu 0,3,4.
  -gputhreads           Cuda thread count. ex: -gputhreads  100 launche 100 threads on selected gpu
  -kernelactivewaiting  Enable active waiting on kernel run.
                        This will raise cpu usage but bring more stability, specially when mining on multiple gpu.
                        WARNING: This affect cpu mining

Network options:
  -s                    Stratum/wallet server address:port.
                        NOTE: You can also use http://address to connect to local wallet.
  -su                   Stratum user
  -pw                   Stratum password
  -fo                   Failover address:port for stratum or local wallet
  -fou                  Failover user for stratum of a local wallet
  -fop                  Failover password for stratum or local wallet
  -r                    Retries connection count for stratum or local wallet
  -dar                  Disable auto-reconnect on connection lost.
                        Note : The miner will exit uppon loosing connection. 

Debug options:
  -testperformance      Run performance test for an amount of seconds
  -testperformancethreads Amount of threads to use for performance test

```

## Examples
```
With config file:
 First use : Edit config.txt and set "s", "su" and desired "cputhreads" or "gputhreads"
 
 Mining with default config.txt : rhminer.exe
 Mining with specific config file : rhminer.exe -configfile {config file pathname}

 With command line:
 Mining solo on cpu          : rhminer.exe -v 2 -r 20 -s http://127.0.0.1:4009 -cpu -cputhreads 4 -extrapayload HelloWorld
 Mining solo on cpu and gpu  : rhminer.exe -v 2 -r 20 -s http://127.0.0.1:4009 -cpu -cputhreads 4 -gpu 0 -gputhreads 262 -extrapayload HelloWorld
 Mining on a pool with 6 gpu : rhminer.exe -v 2 -r 20 -s stratum+tcp://somepool.com:1379 -su MyUsername -gpu 0,1,2,3,4,5 -gputhreads 400,512,512,512,210,512 -extrapayload Rig1
```

## Api access
Default port is 7111.  <br>
Api supports EthMan api format and structure with some limitations. <br>
- At the exception of 'miner_getstat1', the rest of the api needs a pw that match what was passed to rhminer (see -apipw with rhminer and 'Password' field in EthMan)
- For security reasons, method "miner_file" ONLY accept config.txt and config.xml.
  - The config file must be under 8K
  - The config file must be an rhminer compatible config file containing xml data
  - With parameter 'forcerestart' the miner will restart uppon reception of the config file, no mather what was the command line given to rhminer orignialy.
- miner_getstat2 return same as miner_getstat1
- Fan and temperature data are all zero
<br>
To change miner's config remotly, using EthMan, you send a config.txt first then send a restart command to the miner. <br>
    
Just sending empty string will return mining status in json format like that:
```
{
	"infos": [
        {
			"name": "GPU2",
			"threads": 262,
			"speed": 114,
			"accepted": 1,
			"rejected": 0,
			"temp": 0,
			"fan": 0
		}, 
        {
			"name": "CPU",
			"threads": 2,
			"speed": 266,
			"accepted": 3,
			"rejected": 0,
			"temp": 0,
			"fan": 0
		}
	],
	"speed": 380,
	"accepted": 4,
	"rejected": 0,
	"failed": 0,
	"uptime": 91,
	"extrapayload": "",
	"stratum.server": "localhost:4109",
	"stratum.user": "",
	"diff": 0.00000049
}
```
For more details and informations see https://github.com/polyminer1/rhminer/blob/master/Release/API.txt <br>

## Developer Donation
Default donation is 1%. <br>
Donation is hardcoded in the binaries downloadable on gitgub. That is to recoup the 6 month it toke to R&D, develop, stabilize and optimize this miner and for the upcoming bug fixes and many upcoming optimizations. <br>
To disable donation download and compile locally, then use the -devfee option with chosen donation percentage. 0 will disable the donation. <br>

For direct donations:
  * Pascal wallet 529692-23


## Contacts
Discord user ID : polyminer1#8454
Discord channel : https://discord.gg/RVcEpF9 (PascalCoin discord server) <br>
Discord channel : https://discord.gg/Egz2bdS (polyminer1 discord server) <br>
Bitcointalk : https://bitcointalk.org/index.php?topic=5065304.0 <br>
Twitter https://twitter.com/polyminer1 <br>

 