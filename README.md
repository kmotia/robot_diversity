# robot_diversity
This repository houses the data collection and analysis methods for the paper "Effects of Neurological and Morphological Diversity on Robot Swarm Robustness".

## Abstract

Using evolutionary robotics as a medium, we aim to determine if neurological and morphological intra-swarm diversity offers more robustness than other swarm types. Our methods involve the use of an evolutionary algorithm to evolve robot populations, exploring the landscape of possible control strategies and morphologies. We compared three types of robot swarms which can be differentiated by the type of intra-swarm diversity they exhibit: Case 1) No diversity, Case 2) Neurological diversity (diverse controllers), Case 3) Neurological and morphological diversity (diverse controllers and embodiments). We share the results of an experiment comparing the abilities of these three types of robot swarms to navigate a foreign environment post-evolution. Our results show that swarms with neurological and morphological diversity outperform those with neurological diversity, and those with no diversity. Our findings contribute insights into the potential for intra-swarm diversity to be leveraged for the navigation of unknown environments in the real-world.

## To Run

# Set collection parameters. 
- constants.py

# Evolve swarms
- evolveSwarms.py

# Deploy swarms
- playbackSwarms.py

# Analyze data 
- Use the analysis_case1b.py notebook
