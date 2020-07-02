# Collision avoidance strategy for AVs with differnet consumer preference
Vehicle was a fundamental tool of transportation. However, according to the NHTSA report, there were 52,645 fatal crashes and more than 3 million injury crashes involving to the vehicles, most of which were caused by the driver mistakes due to the complex traffic environment. Thus, an advanced safeguarding decision-making system for vehicle to replace the human decisions at the dangerous situation (e.g. potential collision) to avoid the crash is necessary. Besides, with the improvement of passenger demand for vehicle driving quality, we should pay more attention to the passenger comfort. Therefore, a more robustly comprehensive collision avoidance (CA) system should be proposed to meet the demand for passenger (or driver) with different preferences (e.g. aggressive, normal or conservative).

## Approach
To solve the problems mentioned above, we proposed a situation assessment based collision avoidance decision-making framework, which was adaptive to multi-scenario to help the reduction of traffic accidents. Firstly, a probabilistic-model based situation assessment module using conditional random field (CRF), which considered both safety metrics and evasive ability for collision, was proposed to assess the risk of surrounding vehicles. Then, a collision avoidance strategy with different consumer preference was proposed to meet different consumer demand for driving comfort. Finally, we validated our algorithm in a simulator, called Carla. The whole framework was shown below:
<div align=center><img src="picture/CA_strategy.png" width="455"></div>

## Experiments
three crash scenarios respectively called Straight crossing path (SCP), leading vehicle at lower speed (LVLS) and immediate lane change maneuver (ILCM) were introduced here to validate the proposed strategy, see following figture:
<div align=center><img width="500" height="600" src="picture/scenario_setting.png"></div>

## Results with aggressive preference
The experiment recorded some kinematics and risk assessment results of host-vehicle, which were shown following, we could see that the CA strategy tended to take-over the vehicle in a moment of near-collision:
<div align=center><img src="picture/aggressive_results.png"></div>

## Results with consevative preference
The results were shown following, showing different featrues when compared with the aggressive one.
<div align=center><img src="picture/conservative_results.png"></div>

## Comparison of different preferences
Using the normal preference as a baseline, we recorded some metrics which denoted the features of different preferences (i.e. the time-to-collision (TTC) when the CA strategy took over the host-vehicle, the minimum relative distance during experiment and the maximum deceleration taken by the CA strategy)
<div align=center><img width="600" height="400" src="picture/table.png"></div>

## Animation visualization
Some animations were shown following:
### SCP Scenario (from left to right were aggressive preference and conservative preference)
![aggressive preference](picture/gifs/scenario-1/aggressive.gif)
![conservative preference](picture/gifs/scenario-1/conservative.gif)
### LVLS Scenario (from left to right were aggressive preference and conservative preference)
![aggressive preference](picture/gifs/scenario-2/strategy_1-20200316200332.gif)
![conservative preference](picture/gifs/scenario-2/strategy_3-20200316200605.gif)
### ILCM Scenario (from left to right were aggressive preference and conservative preference)
![aggressive preference](picture/gifs/scenario-3/strategy_1-20200316202811.gif)
![conservative preference](picture/gifs/scenario-3/strategy_3-20200316202844.gif) 

