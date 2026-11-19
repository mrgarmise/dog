# FNK0050 visual twin experiment

This additive adapter stacks on the UEL dog simulation draft #3 (c2b2ba71502824a5fe5580c89d1e6253af72d655). It imports the shared research.uel.visual package from Charlie's research/visual-twin branch stacked on #42. Run with both research checkouts on PYTHONPATH. The existing controller, vendor calibration, gait policy and model files are unchanged.

From the Charlie checkout with a sibling dog checkout:

~~~sh
export PYTHONPATH="$PWD:$PWD/../dog/research/fnk0050-sim"
python -m research.uel.visual demo /tmp/dog-run --body dog
python ../dog/research/fnk0050-sim/visual_video.py /tmp/new-dog-frames
python -m pytest ../dog/research/fnk0050-sim/tests -q
~~~

The adapter reads simulated leg link positions and IMU from DogEnv, varies a bounded queue delay and projects the front-left knee marker. The video exporter creates 64 timestamped, hashed, annotated frames, including an occluded marker and simulated image jitter. The independent shared estimator never receives the true delay or hidden joint state. It fits from marker pixels and the known command timeline. The wide video is a separate visual fixture from the close-up fit.

Practice uses 100 control ticks per rollout, perturbed initial roll, inherited mass/inertia/friction/rate variation and the learned delay plus adjacent delays. Each candidate retains its fall, tilt, displacement, clearance and effort evidence. A 0.32 rad release measures modest disturbance recovery; the 0.85 rad unstable-start case must be rejected. This does not demonstrate righting a fallen dog. The tripod candidate needs at least ten samples of elevated front-left foot with three other low feet. Walk acceptance needs positive displacement in every held-out rollout. Each exported candidate has hardware_accepted=false.

See the Charlie branch under research/uel/visual for the offline runbook, protocols, benchmarks, stage evidence, physical review proposal and full test results. Host testing used Python 3.12.14, MuJoCo 4.0.1 and Gymnasium 1.4.0. All 15 dog simulator tests passed, including the existing UEL mapping and adaptation regressions. No changes to a running Pi, deployment, hardware test or authorization are involved.
