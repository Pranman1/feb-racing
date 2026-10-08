Starter packages for `docs/lab-ros-onboarding.md`. Copy them into your `stack/` folder when the
lab says so:

    cp -r labs/onboarding/chatter labs/onboarding/turtle_patrol_interface labs/onboarding/turtle_patrol stack/

`chatter` is a publisher and a subscriber on `/chatter_talk`; `turtle_patrol_interface` defines the
`Patrol` service and `turtle_patrol` offers and calls it. Nothing here is built until it is in `stack/`.
