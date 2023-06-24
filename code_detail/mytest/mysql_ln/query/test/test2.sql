use michen_db;

create table if not exists student(
sid_ int,
name_ varchar(20)
);


alter table student add dep varchar(20);
alter table student change dep depp varchar(20);
alter table student drop depp;
desc student;

insert into student(sid_,name_)
values(1,'陈');

insert into student values(2,'张');
insert into student values(3,'马');

update student set name_ = '红' where sid_ =3;

delete from student where sid_ =1;

insert into student values(4,'天');