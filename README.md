# Saltation_Observer
Work in progress for Hybrid Observer showing the impact of the saltation matrices on a linear observer. We show the importance of the transversality hypothesis alongside the contractivness of two matrices : $M_{\rm before}$ and $M_{\rm after}$

## Requirements
Requires MATLAB 2024b or higher, the [Hybrid Equations Toolbox](https://mathworks.com/matlabcentral/fileexchange/41372-hybrid-equations-toolbox) package.

## Content

The observers.m file is the main script alowing to reproduce all figures.

The /utils folder contains mutiple class definitions.
- BouncingBallSubSystem is a class modelizing bouncing ball as a HybridSubSystem.
- BouncingBallObserver is a class modelizing the constant gain observer as a HybridSubSystem.
- Both K_search files look for appropriate gains regarding the spectral radius of both matrices. One does so by solving LMI while the other performs a naive gridsearch.

## How to reproduce figures

Every computations are made with $x_0 = [5, 2]^\top$.
- Current numerical values $L_c = [0.8, 0.6]^\top, L_d = [0.1, 0.1]^\top, K = [0; 0]^\top$ provide an illustration of local stability of the observer design when all conditions are met. Observer initialized at $\hat{x}_0 = 0.4x_0$.
- $L_c = [0, 0]^\top, L_d = [1, -0.392]^\top$ provide an example of adequate gains for synchronized jumps ($K = [1; 0]^\top, \hat{x}_0 = (1+6.10^{-1})x_0$) that ceases to work for unknown jump time ($K = [0; 0]^\top, \hat{x}_0 = (1+6.10^{-3})x_0$).  $L_d$ was found using the LMI search adapted to the synchronized case. 
- $L_c = [10, 25]^\top, L_d = [0, 0]^\top, K = [3; 0]^\top$ show what happens when transversality of the observer is not met. Observer initialized at $\hat{x}_0 = (1-6.10^{-2})x_0$.
- $L_c = [0.1, 0.25]^\top, L_d = [0.1, 0.1]^\top, K = [0; 0]^\top$ show what happens when only $M_{\rm before}$ is contracting and not $M_{\rm after}$. Observer initialized at $\hat{x}_0 = (1-6.10^{-4})x_0$.
