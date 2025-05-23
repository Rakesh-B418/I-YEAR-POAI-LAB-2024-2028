% -------------------------------------------
% Knowledge Base: Symptoms and Patients
% -------------------------------------------
symptom(john, fever).
symptom(john, cough).
symptom(john, sore_throat).

symptom(mary, rash).
symptom(mary, fever).
symptom(mary, headache).

symptom(sam, cough).
symptom(sam, sore_throat).
symptom(sam, fatigue).

% -------------------------------------------
% Disease Diagnosis Rules
% -------------------------------------------
has_disease(Person, flu) :-
    symptom(Person, fever),
    symptom(Person, cough),
    symptom(Person, sore_throat).

has_disease(Person, measles) :-
    symptom(Person, rash),
    symptom(Person, fever),
    symptom(Person, headache).

has_disease(Person, cold) :-
    symptom(Person, cough),
    symptom(Person, sore_throat),
    symptom(Person, fatigue).

% -------------------------------------------
% Decision Making: Treatment Suggestion
% -------------------------------------------
treatment(flu, 'Take rest, drink fluids, and take paracetamol.').
treatment(measles, 'Consult a doctor, avoid contact, and take vitamin A supplements.').
treatment(cold, 'Drink warm fluids, take rest, and use throat lozenges.').

% -------------------------------------------
% Recommend Treatment Based on Diagnosis
% -------------------------------------------
recommend_treatment(Person, Treatment) :-
    has_disease(Person, Disease),
    treatment(Disease, Treatment).

% -------------------------------------------
% Sample Queries (Use these in SWI-Prolog)
% -------------------------------------------
% ?- has_disease(john, Disease).
% Expected Output: Disease = flu.

% ?- has_disease(mary, Disease).
% Expected Output: Disease = measles.

% ?- has_disease(sam, Disease).
% Expected Output: Disease = cold.

% ?- recommend_treatment(john, Treatment).
% Expected Output: Treatment = 'Take rest, drink fluids, and take paracetamol.'.

% ?- recommend_treatment(mary, Treatment).
% Expected Output: Treatment = 'Consult a doctor, avoid contact, and take vitamin A supplements.'.

% ?- recommend_treatment(sam, Treatment).
% Expected Output: Treatment = 'Drink warm fluids, take rest, and use throat lozenges.'. 5th experiment