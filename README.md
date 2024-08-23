# SimBench

This repo contain data and meta-data required to establish a "judge" LLM (J-LLM) that assesses the quality of Digital Twins (DTs) produced by another LLM. Specifically, SimBench is a benchmark designed to evaluate the proficiency of student large language models (S-LLMs) in generating DTs that can be used in simulators for virtual testing. Given a collection of S-LLMs, this benchmark enables the ranking of the S-LLMs based on their ability to produce high-quality DTs. More than 20 open- and closed-source S-LLMs have been assessed by using a J-LLM produced using the data herein.

Using multi-turn interactions, SimBench employs a rule-based J-LLM that leverages both predefined rules and human-in-the-loop guidance to assign scores for the DTs generated by the S-LLM, thus providing a consistent and expert-inspired evaluation protocol. The J-LLM is specific to a simulator, and herein the proposed benchmarking approach is demonstrated in conjunction with the Chrono multi-physics simulator. Chrono provided the backdrop used to assess an S-LLM in relation to the latter's ability to create digital twins for multibody dynamics, finite element analysis, vehicle dynamics, robotic dynamics, and sensor simulations. The proposed benchmarking principle is broadly applicable and enables the assessment of an S-LLM's ability to generate digital twins for other simulation packages. However, other simulators require different but qualitatively similar data that is specific to the simulator of interest.

A description of the approach used to produce the J-LLM is available here:
```bibtex
@article{jingquanSimbench2024,
  title={{SimBench: A Rule-Based Multi-Turn Interaction Benchmark for Evaluating an LLM's Ability to Generate Digital Twins}},
  author={Jingquan Wang and Harry Zhang and Huzaifa Mustafa Unjhawala and Peter Negrut and Shu Wang and Khailanii Slaton and Radu Serban and Jin-Long Wu and Dan Negrut},
  journal={arXiv preprint arXiv:xxxx.xxxxx},
  year={2024},
  institution={University of Wisconsin-Madison}
}
```
## Highlights of the paper
![SimBench Pipeline](./visualization/demo_overview.svg)
The J-LLM associated with the Chrono SimBench handles multi-physics simulations, including but not limited to:
- **Collision, Contact, and Friction Dynamics (MBD)**: Scenarios involving multi-link arms, gear mechanisms, slider-crank system, and other typical mechanisms.
- **Vibration, deformation, stress, and strain (FEA)**: Scenarios involving cable, beam, shells, plates that evaluate the S-LLM's proficiency in structural analysis.
- **Vehicle Dynamics (VEH)**: City buses, off-road vehicles (e.g., HMMWV, M113), trucks (e.g., Kraz, MAN), and sedans are used to test the S-LLM's ability to simulate driving scenarios. Driver, engine, transmission, and tire models, as well as high-level control policies integrated with sensors, are included in the benchmark.
- **Sensor Integration (SEN)**: Scenarios involving GPS, IMU, LiDAR, and camera sensors are used to exercise the S-LLM's capability to support perception tasks for autonomous vehicles and robotic systems.
- **Robotics Dynamics (RBT)**: The benchmark touches on robotic systems like Turtlebot, Curiosity, and VIPER, as well as granular dynamics and deformable terrain simulations, e.g., the Soil Contact Model (SCM) that come into play in off-road operations for both robots and vehicles.


SimBench draws on 102 demonstration tasks associated with 34 distinct physical systems of the categories MBD through RBT listed above. These tasks involve setting up and progressively modifying digital twins, with each task broken down into three high-quality turns. These turns have been designed by simulation experts to gradually increase in complexity, thus enabling the J-LLM to provide a robust assessment of the S-LLM's capabilities. A list of example simulation scenarios in SimBench is provided in the above figure.


![SimBench Pipeline](./visualization/pipeline_pic.svg)
The SimBench pipeline for evaluating S-LLMs is shown above. The J-LLM is calibrated using a validation set contain pairs of ground truth and generated DTs. The prompts given to the J-LLM are interactively optimized to match the score provided by the expert. Then the J-LLM is used to evaluate the S-LLM based on the generated DTs, ground truth DTs, also the API documentation. 