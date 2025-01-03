<img src="https://github.com/CEINMS-RT/ceinmsrt-core-cpp/blob/main/CEINMS-RT_V2_ICON.png" width="50%" alt="CEINMS-RT logo">

[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)]()

CEINMS-RT [installation](https://ceinms-docs.readthedocs.io/en/latest/Installation%20%5BWindows%5D.html) and [use](https://ceinms-docs.readthedocs.io/en/latest/Tutorial%20%5BWindows%5D%5BUbuntu%5D.html).
Plugin [installation](#installation) and [compilation](https://ceinms-docs.readthedocs.io/en/latest/Compilation%20%5BWindows%5D.html). How to use a [plugin](#how-to-use-it).

# TwinCAT Plugin

Plugin for sending torque from CEINMS-RT to TwinCAT (using ADS).

## Installation

Download and install [Twincat](https://www.beckhoff.com/en-ca/products/automation/twincat/).
In the CEINMS cmake enable the Plugin and then select the Twincat plugin.

## Getting Started

The twincat project can be found in the repo in the VS_2013_project/CEINMS_Twincat folder. This is an example and should be updated for your need (i.e the name and number of joints).
Don't forget to updtae your XML files to use the good joint name and plugin.

## Citation

If you find this repository useful in your research, please consider giving a star ⭐ and cite our [IEEE TBME paper](https://spiral.imperial.ac.uk/bitstream/10044/1/48309/2/durandau%202017.pdf) by using the following BibTeX entries:

```BibTeX
@article{durandau2017robust,
  title={Robust real-time musculoskeletal modeling driven by electromyograms},
  author={Durandau, Guillaume and Farina, Dario and Sartori, Massimo},
  journal={IEEE transactions on biomedical engineering},
  volume={65},
  number={3},
  pages={556--564},
  year={2017},
  publisher={IEEE}
}
```
