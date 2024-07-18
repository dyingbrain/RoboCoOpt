from optimizer_anneal import *

opt=LinkageAnnealer(K=10)

#generate mother having 8 nodes
while True:
    opt.state = opt.generate_init_state()
    if opt.nrN()==5 and opt.check_topology_feasibility() and opt.check_geometry_feasibility():
        mother = opt.state
        break

#generate father having 5 nodes
while True:
    opt.state = opt.generate_init_state()
    if opt.nrN()==4 and opt.check_topology_feasibility() and opt.check_geometry_feasibility():
        father = opt.state
        break

opt.cross_over(mother, father)
print(mother)
print(father)
print(opt.state)