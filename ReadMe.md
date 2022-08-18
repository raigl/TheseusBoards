# Theseus Source Code

The Heinz Nixdorf MuseumsForum ([HNF](https://www.hnf.de/dauerausstellung/ausstellungsbereiche/global-digital/mensch-roboter-leben-mit-kuenstlicher-intelligenz-und-robotik/nachbau-theseus.html))
in Paderborn (Germany) has built a functional replica
of Claude Shannon's mouse maze, named "Theseus",
which was the first known device to explicitly support machine leaning.

To have a prototype quickly running, modern Technology was used,
in particular microcontroller and stepper motors with spindles;
and the latter were shielded from the main controller by a second
microcontroller, and a third one to control the current through
the magnets.
While creating this first prototype, the exact functioning of Shannon's 
machine was better understood, 
and the device now behaves exactly (as far as known) like the one
demonstrated in [Shannon's video demostration](https://techchannel.att.com/playvideo/2010/03/16/In-Their-Own-Words-Claude-Shannon-Demonstrates-Machine-Learning).

The prototype had several drawbacks, but did run well on presentations,
so a second one was build with the same technology due to time and cost
restrictions.

The complete sourcecode for the three Arduino microcontrollers
is provided for those that are interested in this early machine.

Note that a decent replica -- even without relays -- should be more
close to the original, e.g. by using belt drives with collector motors for original speed,
relays to operate the coils and an analog interface to the motors.
Finally, just the relay cicuit could be simulated by micrcontroller,
even if using relays seems to be out of reach.
Note that relay machines are not unreliable and can be repaired 
decades after creation, which is definitely false for any modern logic. 

The licence for the source code is soon supplied; it will be
either public domain or a rather liberal license.





