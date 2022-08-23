Test Cases
=============================================================================================================================================================

| Folder | Target Behaviour Description 																					| Final State of Aircraft																		|
|--------|------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------|
| 0      | Example		       																								|	 																							|
| 1      | Landing performed at LP after single LP found during LZE scan.													| Helicopter on the ground after completing a landing and issuing the mission complete signal.	|
| 2      | Landing performed at LP after multiple LPs found during LZE scan.												| Helicopter on the ground after completing a landing and issuing the mission complete signal.	|
| 3      | Landing performed at LP after single LP found before PLP achieved.												| Helicopter on the ground after completing a landing and issuing the mission complete signal.	|
| 4      | Landing performed at LP after multiple LPs found before PLP achieved.											| Helicopter on the ground after completing a landing and issuing the mission complete signal.	|
| 5      | Landing performed at LP after multiple LPs found during LZE scan but some ignored due to LP accept timer expiry.	| Helicopter on the ground after completing a landing and issuing the mission complete signal.	|
| 6      | Landing performed at LP after multiple LPs found during LZE scan but some ignored due to landing attempt.		| Helicopter on the ground after completing a landing and issuing the mission complete signal.	|
| 7      | Control handed to pilot after no LPs found during LZE scan.														| Helicopter hovering at PLP after BOSS and GCS updates sent. 									|
| 8      | Control handed to pilot after LP reposition timer expired.														| Helicopter hovering at PLP after BOSS and GCS updates sent. 									|
| 9      | Landing performed after new mission loaded during on route phase.												| Helicopter on the ground after completing a landing and issuing the mission complete signal.	|
| 10     | Landing performed after new mission loaded after landing achieved previously.									| Helicopter on the ground after completing a landing and issuing the mission complete signal.	|
| 11     | Landing performed after new mission loaded after pilot handover during LZE scan.									| Helicopter on the ground after completing a landing and issuing the mission complete signal.	|
| 12     | Landing performed after new mission loaded after pilot handover during LP reposition.							| Helicopter on the ground after completing a landing and issuing the mission complete signal.	|