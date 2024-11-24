from apollo_sim.map.map_loader import Map

source_file = '/drive_test/scenario_runner/ApolloSim/data/borregas_ave/base_map.bin'

mp = Map()
mp.parse_from_source(source_file)

backdata = '/Users/mingfeicheng/Desktop/PhD/Github/dev/ApolloSim_private/apollo_sim/map/data/borregas_ave'
mp.export(backdata)