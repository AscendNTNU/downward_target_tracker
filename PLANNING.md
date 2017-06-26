
Slik ser meldingene fra bakkerobot trackeren ut per nå:

```
int32     num_targets    # Defines the number of elements in the arrays below.

int32[]   unique_id      # Each target has a unique ID that persists as long as
                         # the target is tracked.

float32[] position_x     # X and Y values are in the same coordinate frame as
float32[] position_y     # the drone's coordinate frame is relative to: i.e.
float32[] velocity_x     # the grid (with red line = x-axis). In other words
float32[] velocity_y     # position and velocity in the grid plane.

float32[] detection_rate # Detections per second divided by framerate. Flicker
                         # can cause a target to have a rate less than 1. If
                         # it is significantly lower than 1 the target is likely
                         # a false positive and should be ignored.

float     time_until_180 # Seconds remaining until (assumingly) all targets will
                         # do the 180 degree turn. It is corrected live from any
                         # observed turns of tracked targets, and initialized to
                         # 20 seconds when the tracker is started with rosrun.
                         # (Start the tracker at the twenty second mark or otherwise
                         # do not trust this value until the boolean below is true).

bool      observed_180   # True if we have observed any target do a turn so far.
```

Dataene er lagt ut som flere arrays. Du kan iterere over en og en robot slik:

```
downward_target_tracker::tracks msg // From callback
for (int i = 0; i < msg.num_targets)
{
    int id = msg.unique_id[i];
    float x = msg.position_x[i];
    float y = msg.position_y[i];
    float vx = msg.velocity_x[i];
    float vy = msg.velocity_y[i];
    float rate = msg.detection_rate[i];
}
```

Godt å vite:

* Jeg glemmer roboter hvis de ikke har blitt sett på over to sekunder. Dvs. hvis vi ser den igjen etter den er glemt, så vil jeg gi den en ny ```unique_id```.

* Det er støy på hastigheten/retningen forårsaket av gjenskinn fra taket og diverse ting. På fjorårets data var denne støyen i verste fall på cirka 80 grader når det var gjenskinn i midten av plata. I beste fall var den cirka +-5 grader når det ikke var gjenskinn.

* Ved en meter flyhøyde, på fjorårets data, så ser det ut som roboter kan detekteres opp til 1 meter vekk fra dronen, målt langs gulvet.

* Jeg estimerer gjenværende tid til 180 grader snuing per robot jeg tracker ved å detektere tidspunktet når hastigheten snudde. Jeg bruker det til å beregne en gjenværende tid ```time_until_180```, som vi kanskje kan anta holder for alle robotene. Den starter med å være 20 sekunder når trackeren startes, og blir korrigert underveis hvis jeg oppdager at en robot snur mens vi titter på. Flagget ```observed_180``` starter med å være false, og blir true hvis vi har detektert én snuing under programmets livstid.

* Om det ikke funker med en global klokke, så kan dere bruke de individuelle estimatene. Si ifra til meg, fordi jeg publisher ikke de enda.
