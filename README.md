# rhminer 

RandomHash miner for the VNetCoin and PascalCoin blockchain.<br>
Support Intel/AMD 64 bits<br>
Support stratum and solo mining<br>
Works on Windows 7,10, Ubuntu 18, MacOS Intel and AppleSilicon

## Download prebuilt binaries
**Current version is RHMINER_VERSION** <br>

There is one prebuilt binariy per OS architectures. <br>
https://github.com/polyminer1/rhminer/releases/<br>

## Alternative download site : 
* Windows binaries https://mega.nz/#F!DqpAjCJQ!Q12a_YRlu_CWA92kIglKug
* Linux binaries https://mega.nz/#F!Dz4ElAwK!gbWbU4OpmEf6YnOCLIKfSQ


## Mining locally/Solo on VNetCoin
To mine locally/solo you'll need the VNetCoin Node or Wallet {VNetWalletAddress} <br>
In order to mine locally with rhminer, **You need to set a miner name smaller than 16 characters and mine into a private key with encryption type secp256k1**<br>
The best way to assure you're mining in a secp256k1 private key is to create one and select it in the option **"Always mine with this key"**.<br>
**Do not use the "Use random existing key" option** because if one of your key is not a secp256k1 key, the miner will exit when. Plus when there is to much keys in the wallet it gives out errors, sometimes, when submiting nonces<br>
To ensure your miner name is correct, go to Project menu, then Options and set a miner name smaller than 26 characters<br>
To get the number of logical cores, on you system, simply run rhminer with the -completelist option. The last line is the cpu description with the amount of logical core. Ex :<br>

```
Solo mining examples:  rhminer.exe -coin VNET -v 2 -r 20 -s http://127.0.0.1:27001 -cpu -cputhreads 1 -extrapayload HelloWorld
```
Note2: It is not recommended to mine using a laptop.


## Mining locally/Solo on PascalCoin
To mine locally/solo you'll need the official PascalCoin wallet https://github.com/PascalCoin/PascalCoin/releases <br>
In order to mine locally with rhminer, **You need to set a miner name smaller than 26 characters and mine into a private key with encryption type secp256k1**<br>
The best way to assure you're mining in a secp256k1 private key is to create one and select it in the option **"Always mine with this key"**.<br>
**Do not use the "Use random existing key" option** because if one of your key is not a secp256k1 key, the miner will exit when. Plus when there is to much keys in the wallet it gives out errors, sometimes, when submiting nonces<br>
To ensure your miner name is correct, go to Project menu, then Options and set a miner name smaller than 26 characters<br>
To get the number of logical cores, on you system, simply run rhminer with the -completelist option. The last line is the cpu description with the amount of logical core. Ex :<br>
```
Solo mining examples:  rhminer.exe -coin PASC -v 2 -r 20 -s http://127.0.0.1:4009 -cpu -cputhreads 1 -extrapayload HelloWorld

```
Note2: It is not recommended to mine using a laptop.

## Build instructions (Windows)                      
1.  Install VisualStudio 2019 with Platform Toolset v142 named "MSVC v142 - VS 2019 C++ X64/x86 build tools". 
    Make sure you have "Windows 10 SDK" selected.
2.  Download boost_1_72_0 at https://www.boost.org/users/history/version_1_72_0.html
    Unzip at a location of your choice.
    Open VisualStudio and launch a command prompt (Menu Tools|Command Line|Developper Command Prompt)
    From the prompt, cd to the chosen location
    Launch **bootstrap.bat**
    Once finished, compile boost using this command **b2 release debug threading=multi --build-type=complete --toolset=msvc address-model=64 stage**  (NOTE: This can take a long time)
3.  Open solution, select target "Release_CPU" (or "Release_CPU_OLDGEN for intel cpu without sse3)<br>
    Hit compile
4.  Run with proper arguments

## Build Linux (Ubuntu)
sudo apt-get install git build-essential cmake  <br>
Install and compile boost_1_72_0 <br>
Install jsoncpp <br>
git clone https://github.com/polyminer1/rhminer.git <br>
cd rhminer <br>
mkdir build <br>
cd build <br>
*To build for CUDA Pascal :* cmake -DRH_CPU_ONLY=OFF -DRH_DEBUG_TARGET=OFF -DRH_CUDA_ARCH=Pascal --target all ..  <br>
*To build for CPU only    :* cmake -DRH_CPU_ONLY=ON -DRH_DEBUG_TARGET=OFF --target all ..  <br>
make all <br>


## ScreenSaver
To download the screensaver go to release section here https://github.com/polyminer1/rhminer/releases and download **VNetCoinScreenSaver.zip** or **PascalCoinScreenSaver.zip** <br>
To install the screenSaver simply right-click on .scr file and click "install" from the menu. <br>
Then you can configure it. <br>
For Laptop users it is *STROGLY* recommented to set only 1 thread in the scrensaver's config. <br>
 <br>
To set a mining password open regedit.exe and append your password command line, followed by a space, to the string located here : **Computer\HKEY_CURRENT_USER\Software\VNetCoin\ScreenSaver\extra** **Computer\HKEY_CURRENT_USER\Software\PascalCoin\ScreenSaver\extra** <br>
EX: -pw MyEmail@email.com <br>
Dont forget to put a space at the end. <br>


## Stability issues
Thre are some limitations on nvidia gpu to consider.

First, the kernel is not 100% stable in all settings. This mean you'll have to experiment to find the stable sweet spot in term of gputhreads. Maximum thread count does not mean maximum speed. Sometimes lower thread count will give you more stability and more speed also.


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
  -list                 List all gpu and cpu in the system
  -completelist         Exhaustive list of all devices in the system
  -diff                 Set local difficulyu. ex: -diff 999
  -processorsaffinity   On windows only. Force miner to only run on selected logical core processors.
                        ex: -processorsaffinity 0,3 will make the miner run only on logical core #0 and #3.
                        WARNING: Changing this value will affect GPU mining.
  -coin                 Select coin to mine: Supported coins are [VNET, PASC]
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

Gpu options:
  -cpu                  Enable the use of CPU to mine.
                        ex '-cpu -cputhreads 4' will enable mining on cpu while gpu mining.
  -cputhreads           Number of CPU miner threads when mining with CPU. ex: -cpu -cputhreads 4.
                        NOTE: adding + before thread count will disable the maximum thread count safety of one thread per core/hyperthread.
                        Use this option at your own risk.

Network options:
  -dar                  Disable auto-reconnect on connection lost.
                        Note : The miner will exit uppon loosing connection.
  -s                    Stratum/wallet server address:port.
                        NOTE: You can also use http://address to connect to local wallet.
  -su                   Stratum user
  -pw                   Stratum password
  -fo                   Failover address:port for stratum or local wallet
  -fou                  Failover user for stratum of a local wallet
  -fop                  Failover password for stratum or local wallet
  -r                    Retries connection count for stratum or local wallet

Debug options:
  -forcesequentialnonce (For debugging purpose) Force search nonce to be sequential, starting at 0.
                        WARNING: This will gerate alot of uncle and refused solutions.
  -disablecachednoncereuse (For debugging purpose) Disable RandomHash cached nonce reuse.
                        This will lower hashrate substantially.
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
 Mining solo on cpu          : rhminer.exe -v 2 -r 20 -s http://127.0.0.1:27001 -cpu -cputhreads 4 -extrapayload HelloWorld
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
  * VNet wallet xxxx


## Contacts
Discord user ID : polyminer1#8454
Discord channel : https://discord.gg/RVcEpF9 (PascalCoin discord server) <br>
Discord channel : https://discord.gg/Egz2bdS (polyminer1 discord server) <br>
Bitcointalk : https://bitcointalk.org/index.php?topic=5065304.0 <br>
Twitter https://twitter.com/polyminer1 <br>

 