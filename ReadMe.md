# Theseus Source Code

The Heinz Nixdorf MuseumsForum in Paderborn has built a functional replica
of Claude Shannon's mouse maze, named "Theseus"
(See [HNF](https://www.hnf.de/dauerausstellung/ausstellungsbereiche/global-digital/mensch-roboter-leben-mit-kuenstlicher-intelligenz-und-robotik/nachbau-theseus.html).
which was the first known device to explicity support machine leaning.

To have a prototype quickly running, modern Technology was used,
in particular microcontroller and stepper motors with spindles;
and the latter were shielded from the main controller by a second
microcontroller, and a third one to control the current through
the magnets.
The prototype had several drawbacks, but did run well otherwise,
so a second one was build with the same technology.

During this process, the exact functioning of Shannon's was better
understood, and the device now behaves exactly (as far as we
could check) like the one
demonstrated in [Shannon's video demostration](https://techchannel.att.com/playvideo/2010/03/16/In-Their-Own-Words-Claude-Shannon-Demonstrates-Machine-Learning).

The complete sourcecode for the three Arduino microcontrollers
is provided for those that are interested in this early machine.

Note that a decent replica -- even without relays -- should be more
close to the original.

The licence for the source code is soon supplied; it will be
either public domain or a rather liberal license.





