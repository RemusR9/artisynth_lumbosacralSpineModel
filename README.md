# Hybrid lumbosacral spine model in ArtiSynth [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.4453702.svg)](https://doi.org/10.5281/zenodo.4453702) 

This repositry holds a calibrated and validated model of a passive lumbosacral spine based on the data of the [Male Visible Human Project](https://www.nlm.nih.gov/research/visible/visible_human.html). The model is built with the freely available 3D modeling platform [ArtiSynth](https://www.artisynth.org) that supports the combined simulation of multibody and finite element models, together with contact and constraints. As part of an active hybrid model of the human trunk which includes muscles and inverse dynamics, we are continuously improving this model and are open to suggestions for improvements and new potential applications.  

If you use the model or parts of it in your research, please cite the following reference: 
Remus R, Lipphaus A, Neumann M, Bender B. (2021) Calibration and validation of a novel hybrid model of the lumbosacral spine in ArtiSynth – The passive structures. Accepted to PLoS ONE. DOI: 10.1371/journal.pone.0250456.


## Some Details and Tips

More info about the model will follow here soon...

- Always ramp external forces and moments (increase their amplitude evenly distributed over several time steps) to prevent inverted elements in the FE discs. This can be done manually by increasing the numbers and moving the sliders in the simulation control window or automatically by using an input probe - what I would recommend. 
- ...



## License
This source code is licensed under the BSD-3-Clause license found in the [LICENSE file](LICENSE.md) in the root directory of this source tree. 