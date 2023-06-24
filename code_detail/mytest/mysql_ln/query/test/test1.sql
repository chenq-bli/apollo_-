use michen_db;

create table if not exists table1(
sid_ int primary key auto_increment,
name_ varchar(20),
age_ int
)auto_increment=1;

insert into table1 values(NULL,'michen',25);
insert into table1 values(NULL,'zwy',25);
insert into table1 values(NULL,'mzx',25);

alter table table1 modify name_ varchar(30) not null;
insert into table1 values(NULL,null,25);

alter table table1 modify name_ varchar(30) unique;

insert into table1 values(NULL,'mzx',25); 

alter table table1 drop index name_;
insert into table1 values(NULL,'michen',25);

alter table table1 auto_increment=4;

insert into table1 values(NULL,'michen',25);

alter table table1 modify age_ int default 0;
alter table table1 auto_increment=3;
insert into table1(sid_,name_) values(null,'michen');



