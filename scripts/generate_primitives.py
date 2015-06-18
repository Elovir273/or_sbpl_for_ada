#!/usr/bin/env python
import argparse, copy, numpy, random, yaml
from util import *

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description="Generate a primitives for the herb robot")
    parser.add_argument('--resolution', type=float, default=0.1,
                        help="The resolution in xy")
    parser.add_argument('--angles', type=int, default=16,
                        help="The number of angles for the planner to consider")
    parser.add_argument('--max_v', type=float, default=1.0,
                        help="The max forward velocity to allow")
    parser.add_argument('--max_w', type=float, default=0.4,
                        help="The max angular velocity to allow")
    parser.add_argument('--max_t', type=float, default=1.0,
                        help="The max time to allow")
    parser.add_argument('--outfile', type=str, default='primitives.yaml',
                        help="The name of the yaml file to generate")
    parser.add_argument('--samples', type=int, default=300,
                        help="The number of samples to try")
    parser.add_argument('--actions', type=int, default=5,
                        help="The number of actions to select")
    parser.add_argument('--debug', action='store_true',
                        help='Print debug info')
    args = parser.parse_args()

    params = {}
    params['cellsize'] = args.resolution
    params['numangles'] = args.angles
    params['actions'] = []

    wmap = Map(args.resolution, args.angles)

    dt = 0.01
    for idx in range(args.angles):
        start_gc = Coordinate(0, 0, idx)
        start_wc = wmap.GridCoordinateToWorldCoordinate(start_gc)
        start_wc.x = 0
        start_wc.y = 0
               
        # Sample many different controls and pick the one that moves closest
        samples = []
        for sample in range(args.samples):

            # Sample random action parameters
            v = random.uniform(-args.max_v, args.max_v)
            w = random.uniform(-args.max_w, args.max_w)
            t = random.uniform(0, args.max_t)
            
            # Forward simulate
            vm = VehicleModel(start_wc, v, w)
            poses = vm.run(dt, t)

            # Convert to grid coordinate and back to get a distance from the center
            end_wc = copy.deepcopy(poses[-1])
            end_gc = wmap.WorldCoordinateToGridCoordinate(end_wc)
            if end_gc.dist(start_gc) == 0:
                # We didn't move, skip it
                continue

            snapped_wc = wmap.GridCoordinateToWorldCoordinate(end_gc)
            control = Control(v, w, t, end_wc.dist(snapped_wc), poses)
            samples.append(control)
        
        # Now sort all the samples and take the best ones
        sorted_samples = sorted(samples, key=lambda c: c.dist)
        if args.debug:
            print 'Orig: ', [c.dist for c in samples]
            print 'Sorted: ', [c.dist for c in sorted_samples]

        action = {}
        action['angle'] = idx
        action['poses'] = []
        for j in range(args.actions):
            sample = sorted_samples[j]
            pose_list = []
            for p in sample.poses:
                pose_list.append([float(p.x), float(p.y), float(p.theta)])
            action['poses'].append(pose_list)
        
        params['actions'].append(action)
            

    with open(args.outfile, 'w') as f:
        f.write(yaml.dump(params, default_flow_style=False))

    print 'Wrote primitives to file %s' % args.outfile
